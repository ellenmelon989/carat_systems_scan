"""
ir_reader_base.py — shared interface for all IR temperature reader backends.

Every backend (REST API, OptoMMP, direct tap, mock) implements this same
interface so scan_manager.py doesn't care which one is plugged in.
"""

from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Optional


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

    def close(self):
        pass
