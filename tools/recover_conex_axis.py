"""Controlled open-loop recovery for a CONEX-AGAP axis outside SL/SR.

This utility is intentionally separate from the normal motion controller.
The normal driver refuses to construct while either encoder lies outside the
stored software limits, and PA/PR closed-loop commands cannot safely recover
that condition.  Newport documents XU/XR as the open-loop stepping path.

Running this file without ``--execute`` is read-only.  Execution requires an
explicit axis, a small signed XU amplitude, a bounded positive step count, and
an exact confirmation phrase.  One invocation commands only one batch, reads
the encoder before and after, restores the original XU working value, and
reports whether the movement was toward the allowed range.

The XU amplitude is not an angular unit and has no fixed relationship to
motion.  Start with a small trial; never infer a large batch from nominal
stage sensitivity alone.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from motion.real_conexagap_motion import (
    _BAUD_RATE,
    _BYTESIZE,
    _PARITY,
    _STOPBITS,
    _TERMINATOR,
    _XONXOFF,
    probe_conex_connection,
)

import serial


_CONFIRMATION = "RECOVER-OUT-OF-LIMIT-AXIS"
_READY_STATES = {"32", "33", "34", "35", "36"}
_MOVING_STATES = {"28", "29", "46"}
_MAX_TRIAL_AMPLITUDE = 10
_MAX_TRIAL_STEPS = 100
_POLL_S = 0.02
_MOVE_TIMEOUT_S = 15.0


def _load_connection(config_path: str) -> tuple[str, int, float]:
    with open(config_path, encoding="utf-8-sig") as stream:
        config = yaml.safe_load(stream)
    motion = config.get("motion", {})
    port = motion.get("port")
    if not port:
        raise ValueError(f"No motion.port is configured in {config_path!r}.")
    address = int(motion.get("controller_address", 1))
    timeout = float(motion.get("serial_timeout_s", 2.0))
    return str(port), address, timeout


def _query(ser, command: str) -> str:
    ser.reset_input_buffer()
    ser.write((command + _TERMINATOR).encode("ascii"))
    ser.flush()
    raw = ser.readline()
    if not raw:
        raise RuntimeError(f"No response to {command!r}.")
    response = raw.decode("ascii", errors="replace").strip()
    prefix = command[:-1] if command.endswith("?") else command
    if not response.upper().startswith(prefix.upper()):
        raise RuntimeError(
            f"Unexpected response {response!r} to {command!r}."
        )
    return response[len(prefix):].strip()


def _send(ser, command: str):
    ser.write((command + _TERMINATOR).encode("ascii"))
    ser.flush()


def _state(ser, address: int) -> str:
    response = _query(ser, f"{address}TS")
    if len(response) < 2:
        raise RuntimeError(f"Malformed TS response: {response!r}")
    return response[-2:].upper()


def _axis_snapshot(probe: dict, axis: str) -> tuple[float, float, float]:
    key = axis.lower()
    return (
        float(probe[f"position_{key}_deg"]),
        float(probe[f"negative_limit_{key}_deg"]),
        float(probe[f"positive_limit_{key}_deg"]),
    )


def _needed_direction(position: float, negative: float, positive: float) -> int:
    if position < negative:
        return 1
    if position > positive:
        return -1
    return 0


def _directional_xu_value(response: str, requested_amplitude: int) -> int:
    """Select the stored XU value for the direction this trial changes.

    CONEX-AGAP V2.0.3 returns both directional working values from XU? as
    e.g. ``-35,+35``.  Some documentation/examples show only one integer, so
    accept that form too.  Restoring only the same-sign value leaves the
    opposite direction untouched.
    """
    try:
        values = [int(part.strip()) for part in response.split(",")]
    except ValueError as exc:
        raise RuntimeError(f"Malformed XU response: {response!r}") from exc

    if requested_amplitude > 0:
        candidates = [value for value in values if value > 0]
    else:
        candidates = [value for value in values if value < 0]

    if len(candidates) == 1:
        return candidates[0]
    if len(values) == 1:
        # Compatibility with firmware that reports one current value.
        return values[0]
    raise RuntimeError(
        f"XU response {response!r} did not contain exactly one value for "
        f"the {'positive' if requested_amplitude > 0 else 'negative'} "
        "direction."
    )


def _print_probe(probe: dict):
    print("CONEX RECOVERY PREFLIGHT (read-only)")
    print(f"  Port:       {probe['port']}")
    print(f"  Controller: {probe['revision']}")
    print(f"  Stage:      {probe['stage_id']}")
    print(
        f"  State:      {probe['controller_state']} "
        f"({probe['controller_state_name']})"
    )
    for axis in ("U", "V"):
        position, negative, positive = _axis_snapshot(probe, axis)
        direction = _needed_direction(position, negative, positive)
        status = "inside limits"
        if direction > 0:
            status = "outside LOW; encoder must increase"
        elif direction < 0:
            status = "outside HIGH; encoder must decrease"
        print(
            f"  {axis}: {position:.6f} deg; limits "
            f"[{negative:.6f}, {positive:.6f}] -- {status}"
        )
    print("  Motion:     none commanded")


def _execute_trial(
    port: str,
    address: int,
    timeout: float,
    axis: str,
    amplitude: int,
    steps: int,
    before_probe: dict,
) -> tuple[float, float]:
    before, negative, positive = _axis_snapshot(before_probe, axis)
    required_direction = _needed_direction(before, negative, positive)
    if required_direction == 0:
        raise RuntimeError(
            f"Axis {axis} is already within its stored limits; recovery "
            "motion is refused."
        )

    ser = serial.Serial(
        port=port,
        baudrate=_BAUD_RATE,
        bytesize=_BYTESIZE,
        parity=_PARITY,
        stopbits=_STOPBITS,
        xonxoff=_XONXOFF,
        timeout=timeout,
        write_timeout=timeout,
    )
    original_amplitude = None
    try:
        state = _state(ser, address)
        if state not in _READY_STATES:
            raise RuntimeError(
                f"Controller state is {state}, not READY. The recovery "
                "utility will not enable or change controller state."
            )

        original_response = _query(ser, f"{address}XU{axis}?")
        original_amplitude = _directional_xu_value(
            original_response, amplitude
        )
        _send(ser, f"{address}XU{axis}{amplitude}")
        error = _query(ser, f"{address}TE")
        if error != "@":
            raise RuntimeError(f"CONEX rejected XU setting: TE={error!r}")

        _send(ser, f"{address}XR{axis}{steps}")
        deadline = time.monotonic() + _MOVE_TIMEOUT_S
        while time.monotonic() < deadline:
            state = _state(ser, address)
            if state not in _MOVING_STATES:
                break
            time.sleep(_POLL_S)
        else:
            _send(ser, f"{address}ST")
            raise RuntimeError(
                f"Open-loop trial did not finish within {_MOVE_TIMEOUT_S:g}s; "
                "ST was sent. Rerun the read-only diagnostic before doing "
                "anything else."
            )

        after = float(_query(ser, f"{address}TP{axis}"))
    finally:
        if original_amplitude is not None:
            try:
                if _state(ser, address) in _READY_STATES:
                    _send(ser, f"{address}XU{axis}{original_amplitude}")
            except Exception:
                print(
                    "WARNING: could not restore the original temporary XU "
                    "working value. It resets on controller power-cycle."
                )
        ser.close()

    delta = after - before
    moved_toward = delta * required_direction > 0
    print("CONEX RECOVERY TRIAL RESULT")
    print(f"  Axis:       {axis}")
    print(f"  Before:     {before:.6f} deg")
    print(f"  After:      {after:.6f} deg")
    print(f"  Delta:      {delta:+.6f} deg")
    print(f"  Limits:     {negative:.6f} to {positive:.6f} deg")
    print(f"  Direction:  {'TOWARD limits' if moved_toward else 'NOT toward limits'}")
    print(f"  Command:    XU={amplitude}, XR steps={steps}")
    print("  Automatic repeat: none")
    if not moved_toward:
        print("  WARNING: do not repeat this sign; rerun read-only preflight.")
    return before, after


def main():
    parser = argparse.ArgumentParser(
        description="Small, explicitly confirmed CONEX open-loop recovery trial"
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--axis", choices=["U", "V"])
    parser.add_argument("--amplitude", type=int)
    parser.add_argument("--steps", type=int)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--confirm",
        help=f"Required with --execute; must equal {_CONFIRMATION!r}",
    )
    args = parser.parse_args()

    if not Path(args.config).is_file():
        parser.error(f"config file not found: {args.config}")

    port, address, timeout = _load_connection(args.config)
    probe = probe_conex_connection(port, address, timeout)
    _print_probe(probe)

    if not args.execute:
        return

    if args.axis is None or args.amplitude is None or args.steps is None:
        parser.error("--execute requires --axis, --amplitude, and --steps")
    if args.confirm != _CONFIRMATION:
        parser.error(
            f"--execute requires --confirm {_CONFIRMATION} exactly"
        )
    if args.amplitude == 0 or abs(args.amplitude) > _MAX_TRIAL_AMPLITUDE:
        parser.error(
            f"trial amplitude must be non-zero and between "
            f"-{_MAX_TRIAL_AMPLITUDE} and +{_MAX_TRIAL_AMPLITUDE}"
        )
    if not 1 <= args.steps <= _MAX_TRIAL_STEPS:
        parser.error(f"trial steps must be 1-{_MAX_TRIAL_STEPS}")

    print()
    print("EXECUTING ONE OPEN-LOOP RECOVERY TRIAL")
    _execute_trial(
        port,
        address,
        timeout,
        args.axis,
        args.amplitude,
        args.steps,
        probe,
    )


if __name__ == "__main__":
    main()
