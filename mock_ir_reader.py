"""
mock_ir_reader.py — synthetic IR reader for dev/testing without any
hardware connected. Use this to build and test scan_manager.py's Stage 2/3
scan logic while the real backend (REST/OptoMMP/direct tap) gets sorted out.
"""

import time
import random

from ir_reader_base import IRReader, IRReading


class MockIRReader(IRReader):
    def __init__(self, base_temp_c: float = 850.0, noise_c: float = 0.3,
                 emissivity: float = 0.85):
        self.base_temp_c = base_temp_c
        self.noise_c = noise_c
        self.emissivity = emissivity

    def read(self) -> IRReading:
        read_time = time.time()
        value = self.base_temp_c + random.uniform(-self.noise_c, self.noise_c)
        return IRReading(value_c=value, emissivity=self.emissivity,
                          pac_timestamp=read_time, read_time=read_time, stale=False)


if __name__ == "__main__":
    reader = MockIRReader()
    for _ in range(5):
        print(reader.read())
        time.sleep(1)
