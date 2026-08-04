"""
run_gui.py

Entry point for the carat_scanner GUI. Mirrors scan_manager.py's own
--config argparse pattern. Run from the repo root:

    python run_gui.py [--config config.yaml]

so the flat `from motion...` / `from readers...` / `from gui...`
imports resolve exactly the way they already do for every other
script in this repo (no package install, no sys.path hacking --
namespace packages resolved relative to cwd).
"""

import argparse

import yaml


def _print_conex_probe(config):
    """Run the program's no-motion CONEX connection check and print it."""
    from motion.real_conexagap_motion import probe_conex_connection

    motion_cfg = config.get("motion", {})
    if motion_cfg.get("controller") != "conex_agap":
        raise ValueError(
            "motion.controller must be 'conex_agap' in config.yaml before "
            "testing the CONEX connection."
        )

    result = probe_conex_connection(
        port=motion_cfg.get("port"),
        address=motion_cfg.get("controller_address", 1),
        serial_timeout=motion_cfg.get("serial_timeout_s", 2.0),
    )

    print("CONEX CONNECTION: PASS")
    print(f"  Port:       {result['port']}")
    print(f"  Address:    {result['address']}")
    print(f"  Controller: {result['revision']}")
    print(f"  Stage:      {result['stage_id']}")
    print(f"  Position U: {result['position_u_deg']:.6f} deg")
    print(f"  Position V: {result['position_v_deg']:.6f} deg")
    print(f"  Target U:   {result['target_u_deg']:.6f} deg")
    print(f"  Target V:   {result['target_v_deg']:.6f} deg")
    print(
        f"  State:      {result['controller_state']} "
        f"({result['controller_state_name']})"
    )
    print(
        f"  U limits:   {result['negative_limit_u_deg']:.6f} to "
        f"{result['positive_limit_u_deg']:.6f} deg"
    )
    print(
        f"  V limits:   {result['negative_limit_v_deg']:.6f} to "
        f"{result['positive_limit_v_deg']:.6f} deg"
    )
    if not (
        result["negative_limit_u_deg"]
        <= result["position_u_deg"]
        <= result["positive_limit_u_deg"]
    ):
        print("  WARNING:    U position is outside the stored U software limits")
    if not (
        result["negative_limit_v_deg"]
        <= result["position_v_deg"]
        <= result["positive_limit_v_deg"]
    ):
        print("  WARNING:    V position is outside the stored V software limits")
    print("  Motion:     none commanded")


def main():
    parser = argparse.ArgumentParser(
        description="Launch the carat_scanner GUI, seeded from a config YAML.",
    )
    parser.add_argument(
        "--config", default="config.yaml",
        help="Path to the config YAML to load as the GUI's starting values (default: config.yaml).",
    )
    parser.add_argument(
        "--test-conex", action="store_true",
        help=(
            "Test the configured CONEX serial connection and exit. Queries "
            "identity, state, position, target, and software limits only; "
            "does not enable, home, or move it."
        ),
    )
    args = parser.parse_args()

    # utf-8-sig: tolerates (and strips) a UTF-8 BOM if one is present,
    # transparent no-op if not -- config.yaml is a hand-edited file an
    # operator may open/save in Notepad on Windows, which can add a BOM.
    # A BOM landing on the first key (e.g. "﻿scan:") makes
    # yaml.safe_load either error or silently misparse the top-level key,
    # so config["scan"] etc. below would KeyError. gui/calibration_panel.py
    # and scan/calibrate_scan_area.py already guard against this when
    # re-reading config.yaml after a Calibrate-tab write; this is the same
    # guard applied to every other place config.yaml gets read, including
    # this one -- the GUI's own entry point, which previously did not have it.
    with open(args.config, encoding="utf-8-sig") as f:
        config = yaml.safe_load(f)

    if args.test_conex:
        try:
            _print_conex_probe(config)
        except Exception as exc:
            parser.exit(
                1,
                "CONEX CONNECTION: FAILED\n"
                f"  {exc}\n\n"
                "Close the Newport applet or any other program using the "
                "COM port, confirm motion.port in config.yaml, and try again.\n",
            )
        return

    # Importing the GUI pulls in the plotting and sensor stack.  Keep it out
    # of --test-conex so a missing unrelated spectrometer/plot dependency
    # cannot obscure the serial connection result.
    from gui.app import App

    # config_path is threaded through to the Calibrate tab (see
    # gui/calibration_panel.py) so it can patch the SAME file that was
    # just loaded here, exactly like calibrate_scan_area.py's own
    # `python calibrate_scan_area.py [config.yaml]` usage.
    app = App(config, args.config)
    app.mainloop()


if __name__ == "__main__":
    main()
