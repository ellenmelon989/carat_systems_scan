"""
ir_reader_base.py — shared interface for all IR temperature reader backends.

Every backend (REST API, OptoMMP, direct tap, mock) implements this same
interface so scan_manager.py doesn't care which one is plugged in.
"""

from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class IRReading:
    value_c: float
    emissivity: float
    pac_timestamp: Optional[float]   # epoch seconds; None if read failed
    read_time: float                 # epoch seconds, wall clock at read
    stale: bool                      # True if value/timestamp can't be trusted
    error: Optional[str] = None


class IRReader:
    """Base interface. All backends implement read()."""

    def read(self) -> IRReading:
        raise NotImplementedError

    def read_averaged(self, averaging_time_s: float,
                      poll_interval_s: float = 0.1,
                      min_reads: int = 3) -> Tuple[float, IRReading]:
        """
        Poll read() for averaging_time_s, return (mean_value_c, last_reading).
        Skips stale/errored readings. Returns NaN if no valid reads collected.
        """
        valid_values = []
        last_reading = None
        t_end = time.time() + averaging_time_s

        while time.time() < t_end or len(valid_values) < min_reads:
            reading = self.read()
            last_reading = reading
            if not reading.stale and reading.error is None:
                valid_values.append(reading.value_c)
            if time.time() >= t_end and len(valid_values) >= min_reads:
                break
            time.sleep(poll_interval_s)

        mean_val = float(sum(valid_values) / len(valid_values)) if valid_values else float("nan")
        return mean_val, last_reading

    def close(self):
        pass


def get_ir_reader(config) -> "IRReader":
    """
    Factory. Returns the appropriate IRReader based on config["ir"]["source"]:
      "pac"   → RestApiIRReader (live PAC REST API)
      "mock"  → MockIRReader
      None    → MockIRReader
    """
    source = config.get("ir", {}).get("source")

    if source == "pac":
        from .rest_ir_reader import RestApiIRReader
        ir_cfg = config["ir"]["pac"]
        return RestApiIRReader(
            controller_ip=ir_cfg["ip"],
            api_key_id=ir_cfg["api_key_id"],
            api_key_value=ir_cfg["api_key_value"],
            variable_name=ir_cfg["tag_name"],
        )

    if source not in (None, "mock"):
        raise ValueError(f"Unknown ir.source: {source!r}. Expected 'pac' or 'mock'.")

    from .mock_ir_reader import MockIRReader
    return MockIRReader()
