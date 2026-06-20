"""
direct_tap_ir_reader.py — reads the Williamson transmitter directly,
bypassing the PAC entirely. LAST RESORT.

Only use this if Carat won't grant REST or OptoMMP access to the PAC at all.
Unlike the other two backends, this is a hardware change (new wiring/tap),
not just software.

The transmitter is almost certainly putting out ONE of:
  (a) 4-20 mA analog loop -> needs your own USB DAQ wired in parallel with
      the existing PAC analog input (read-only tap, don't break the existing
      loop). Scale current -> temperature using the transmitter's documented
      span. Not implemented below -- swap in your DAQ vendor's analog-read
      call if this turns out to be the actual signal type.
  (b) RS-485/Modbus RTU -> cleaner, no current-loop splicing required, but
      only works if the transmitter actually exposes a Modbus register.

This file implements (b), the lower-risk wiring change. CHECK THE WILLIAMSON
TRANSMITTER'S MODEL NUMBER AND DATASHEET before using this -- the register
address, slave address, and scale factor below are placeholders, not real
values.

Requires: pip install pymodbus
"""

import time

from ir_reader_base import IRReader, IRReading


class DirectTapIRReader(IRReader):
    def __init__(self, serial_port: str, modbus_address: int, register: int,
                 baudrate: int = 9600, emissivity: float = 0.0, scale: float = 0.1):
        from pymodbus.client import ModbusSerialClient  # lazy import
        self.client = ModbusSerialClient(port=serial_port, baudrate=baudrate, timeout=1)
        self.modbus_address = modbus_address
        self.register = register
        self.scale = scale  # raw register units -> deg C; confirm in transmitter manual
        self.emissivity = emissivity

    def read(self) -> IRReading:
        read_time = time.time()
        if not self.client.connect():
            return IRReading(value_c=float("nan"), emissivity=self.emissivity,
                              pac_timestamp=None, read_time=read_time, stale=True,
                              error="modbus connect failed")
        try:
            result = self.client.read_holding_registers(
                self.register, count=1, slave=self.modbus_address)
            if result.isError():
                raise IOError(str(result))
            raw = result.registers[0]
            value = raw * self.scale
            return IRReading(value_c=value, emissivity=self.emissivity,
                              pac_timestamp=read_time, read_time=read_time, stale=False)
        except Exception as e:
            return IRReading(value_c=float("nan"), emissivity=self.emissivity,
                              pac_timestamp=None, read_time=read_time, stale=True,
                              error=str(e))

    def close(self):
        self.client.close()


if __name__ == "__main__":
    # PLACEHOLDERS -- confirm against the transmitter's actual datasheet
    # before trusting any value this returns.
    reader = DirectTapIRReader(
        serial_port="COM_PORT_HERE",
        modbus_address=1,
        register=0,
        scale=0.1,
    )
    print(reader.read())
