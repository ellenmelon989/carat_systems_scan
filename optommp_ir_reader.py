"""
optommp_ir_reader.py — Opto 22 OptoMMP backend. FIRMWARE-AGNOSTIC FALLBACK.

Use this if the controller's firmware is below R9.5a (REST API requirement)
or if Carat's policy won't allow the REST/HTTP server to be enabled.

!!! base_address below is a PLACEHOLDER. You must confirm it against:
    OptoMMP Protocol Guide, Form 1465 (PAC Control strategy variable
    tables section):
    https://www.opto22.com/support/resources-tools/documents/1465-optommp-protocol-guide
combined with this variable's table index in PAC Control (Debug menu ->
Inspect the float variable -> it shows its position in the float table).
Wrong address = silently wrong data, not an error -- don't guess this part.

The request/response byte structure below (tcode 5 = read block request,
16-byte request, 24-byte response, data in bytes 16:20) is taken directly
from Opto 22's own published Python OptoMMP example -- that part is solid
and shouldn't need changes.

No external dependencies (uses stdlib socket + struct only).
"""

import time
import socket
import struct
from typing import Optional

from ir_reader_base import IRReader, IRReading


class OptoMmpIRReader(IRReader):
    READ_BLOCK_TCODE = 5
    PORT = 2001

    def __init__(self, controller_ip: str, base_address: int, table_index: int,
                 emissivity: float = 0.0, timeout: float = 2.0):
        self.host = controller_ip
        self.address = base_address + (table_index * 4)  # float32 = 4 bytes/slot
        self.emissivity = emissivity
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def _connect(self):
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.PORT))

    def _build_read_request(self, dest: int, size: int = 4) -> bytes:
        addr_bytes = dest.to_bytes(4, byteorder="big")
        return bytes([
            0, 0, (1 << 2), (self.READ_BLOCK_TCODE << 4),
            0, 0, 255, 255,
            addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
            0, size, 0, 0,
        ])

    def read(self) -> IRReading:
        read_time = time.time()
        try:
            self._connect()
            self.sock.send(self._build_read_request(self.address))
            data = self.sock.recv(24)
            data_block = data[16:20]
            value = struct.unpack(">f", data_block)[0]
            return IRReading(value_c=value, emissivity=self.emissivity,
                              pac_timestamp=read_time, read_time=read_time, stale=False)
        except (socket.error, struct.error) as e:
            self.sock = None  # force reconnect on next read
            return IRReading(value_c=float("nan"), emissivity=self.emissivity,
                              pac_timestamp=None, read_time=read_time, stale=True,
                              error=str(e))

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None


if __name__ == "__main__":
    # Fill in base_address (from Form 1465) and table_index (from PAC Control's
    # Inspect view) before this will return real data.
    reader = OptoMmpIRReader(
        controller_ip="CONTROLLER_IP_HERE",
        base_address=0x00000000,  # PLACEHOLDER -- look this up, see docstring
        table_index=0,             # PLACEHOLDER -- this variable's slot in the float table
    )
    print(reader.read())
