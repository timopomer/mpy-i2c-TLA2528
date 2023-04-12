from machine import Pin, I2C
from time import sleep, sleep_ms

SYSTEM_STATUS_ADDRESS = 0x00
READ_BYTE = 0b00010000
DEBUG_MODE = True

def bytes_to_bits(bytes_object):
    bit_list = []
    for byte in bytes_object:
        for i in range(7, -1, -1):
            bit_list.append((byte >> i) & 1)
    return bit_list


class TLA2528SystemStatus:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    @property
    def brown_out_reset(self) -> bool:
        result = self._response_byte & 0x01
        return bool(result)

    @property
    def high_speed_mode(self) -> bool:
        result = (self._response_byte & 0x20) >> 5
        return bool(result)

    @property
    def sequencer_in_progress(self) -> bool:
        result = (self._response_byte & 0x40) >> 6
        return bool(result)


class TLA2528:

    def __init__(self, i2c: I2C, device_addr: int) -> None:
        self._i2c = i2c
        self._device_addr = device_addr

    def read_bytes(self, register_address: int, bytecount: int=1):
        write_request = bytes([READ_BYTE, register_address])
        i2c.writeto(device, write_request)
        read_bytes = i2c.readfrom(device, bytecount)
        if DEBUG_MODE:
            print(f"write req: {bytes_to_bits(write_request)}")
            print(f"read bytes: {bytes_to_bits(read_bytes)}")
        return read_bytes

    def get_system_status(self) -> TLA2528SystemStatus:
        # v = reg_read(i2c, device, SYSTEM_STATUS_ADDRESS, nbytes=1)
        # sys_status_cmd = bytearray([SYSTEM_STATUS_ADDRESS])
        # sleep_ms(10)  # Wait for the chip to process the command
        read = self.read_bytes(SYSTEM_STATUS_ADDRESS)
        return TLA2528SystemStatus(read)


pin = Pin("LED", Pin.OUT)
pin.toggle()


i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)

while True:
    devices = i2c.scan()
    print(devices)
    if devices:
        device = devices[0]
        break
    else:
        print("no devices found")
# Send the System Status command to the TLA2528 chip and read the response byte
chip = TLA2528(i2c, device)
b = 0
while True:
    status = chip.get_system_status()
    print(status.brown_out_reset)
    print(status.high_speed_mode)
    print(status.sequencer_in_progress)
    print('-------------')
    b+=1
    print(b)
    pin.toggle()
    sleep(1)
