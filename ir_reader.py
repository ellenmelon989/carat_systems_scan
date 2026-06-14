"""
ir_reader.py

Responsible for reading IR temperature data, preferentially from
the Opto 22 SNAP PAC R1 (if the Williamson pyrometer value is
already exposed as a PAC tag), with a real-hardware implementation
to be filled in once the tag/connection details are confirmed.

Provides: IRReader (abstract), MockIRReader, PACIRReader (skeleton).
"""

from abc import ABC, abstractmethod
import random
import time


class IRReader(ABC):
    """Abstract interface for IR temperature acquisition."""

    @abstractmethod
    def read_raw(self):
        """Return a single raw IR reading (temperature or signal)."""
        raise NotImplementedError

    def read_averaged(self, averaging_time_s, sample_interval_s=0.1):
        """
        Sample read_raw() repeatedly over averaging_time_s and
        return (averaged_value, list_of_raw_samples).

        On any read failure, the failing sample is skipped; if all
        samples fail, raises RuntimeError so the caller can apply
        the configured error policy (NaN + flag, retry, etc.).
        """
        samples = []
        n_samples = max(1, int(averaging_time_s / sample_interval_s))

        for _ in range(n_samples):
            try:
                samples.append(self.read_raw())
            except Exception as e:
                print(f"[IRReader] Sample failed: {e}")
            time.sleep(sample_interval_s)

        if not samples:
            raise RuntimeError("All IR samples failed")

        return sum(samples) / len(samples), samples


class MockIRReader(IRReader):
    """Simulated IR reader for development without hardware."""

    def __init__(self, base_temp_c=900.0, noise_c=2.0):
        self.base_temp_c = base_temp_c
        self.noise_c = noise_c

    def read_raw(self):
        return self.base_temp_c + random.uniform(-self.noise_c, self.noise_c)


class PACIRReader(IRReader):
    """
    Reads IR temperature from a tag on the Opto 22 SNAP PAC R1.

    TODO (Phase 2):
    - Confirm PAC IP address
    - Confirm tag name bound to the Williamson pyrometer analog input
    - Confirm units (raw counts vs scaled engineering units / deg C)
    - Choose access protocol: PAC REST API (R9.5a+) or OptoMMP
    - Implement read_raw() to query the tag value
    - Ensure access is READ-ONLY (do not write to PAC tags)
    """

    def __init__(self, ip_address, tag_name, **kwargs):
        self.ip_address = ip_address
        self.tag_name = tag_name
        if ip_address is None or tag_name is None:
            raise NotImplementedError(
                "PAC IP address and tag name must be confirmed on-site "
                "before PACIRReader can be used (see Phase 2)."
            )
        # TODO: establish connection (REST session or OptoMMP client)

    def read_raw(self):
        # TODO: query self.tag_name from the PAC and return its value
        raise NotImplementedError("PACIRReader.read_raw not yet implemented - Phase 2")


def get_ir_reader(config):
    """
    Factory function. Returns a MockIRReader unless config
    specifies a working PAC connection.
    """
    ir_cfg = config.get("ir", {})
    source = ir_cfg.get("source", "mock")

    if source == "pac":
        pac_cfg = ir_cfg.get("pac", {})
        try:
            return PACIRReader(ip_address=pac_cfg.get("ip"), tag_name=pac_cfg.get("tag_name"))
        except NotImplementedError as e:
            print(f"[ir_reader] {e}\nFalling back to MockIRReader.")
            return MockIRReader()

    return MockIRReader()


if __name__ == "__main__":
    reader = MockIRReader()
    avg, samples = reader.read_averaged(averaging_time_s=1.0, sample_interval_s=0.2)
    print(f"Averaged: {avg:.2f}, samples: {samples}")
