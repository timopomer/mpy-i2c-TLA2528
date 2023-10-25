from machine import Pin, I2C, UART
from time import sleep, sleep_ms

DEBUG_MODE = True

def adc_to_voltage(adc_bytes: bytes, avdd=5.0, n_bits=12):
    # Calculate the voltage per least significant bit (LSB)
    adc_value = int.from_bytes(adc_bytes, 'big')
    adc_value = adc_value >> 4

    voltage_per_lsb = avdd / (2 ** n_bits)
    # Convert the 2-byte ADC value to voltage
    voltage = adc_value * voltage_per_lsb
    return voltage

def create_bitmask(size: int, offset: int) -> int:
    mask = ((1 << size) - 1) << offset
    return mask

def replace_bits(original: int, replacement: int, size: int, offset: int, original_size: int = 8) -> int:
    if replacement >= (1 << size):
        raise ValueError(f"Replacement value {replacement} is too large for the specified size {size}")

    if offset + size > original_size:
        raise ValueError(f"Offset {offset} and size {size} exceed the original size {original_size}")

    original_mask = ~create_bitmask(size, offset)
    replacement_mask = replacement << offset

    return (original & original_mask) | replacement_mask

def bytes_to_bits(bytes_object: bytes):
    bit_list = []
    for byte in bytes_object:
        for i in range(7, -1, -1):
            bit_list.append((byte >> i) & 1)
    return f"bits: {''.join([str(b) for b in bit_list])}"

def format_register(name: str, field_configs) -> str:
    def int_to_bit_str(value: int, length: int) -> str:
        return ', '.join(str(int((value >> i) & 1)) for i in range(length - 1, -1, -1))

    field_configs = field_configs[::-1]
    total_bits = sum(bit_length for _, _, bit_length in field_configs)
    if total_bits is not 8:
        raise IndexError()
    
    formatted_values = [int_to_bit_str(value, bit_length) if isinstance(value, int) else str(value)
                        for _, value, bit_length in field_configs]
    
    max_len = max(max(len(field_name), len(str(value))) for field_name, value, _ in field_configs)

    num_fields = len(field_configs)
    separator = f"+{'-' * (max_len + 2)}" * num_fields + "+\n"

    formatted_field_names = " | ".join(f"{field_name:<{max_len}}" for field_name, _, _ in field_configs)
    formatted_values_str = " | ".join(f"{value:<{max_len}}" for value in formatted_values)

    table = f"{name}\n" + separator
    table += f"| {formatted_field_names} |\n" + separator
    table += f"| {formatted_values_str} |\n" + separator
    
    return table

class TLA2528OversamplingRatioConfiguration:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def __repr__(self) -> str:
        field_configs = [
            ("OSR[2:0]", 3, 3),
            ("RESERVED[4:0]", 0, 5),  # Reserved bits
        ]
        return format_register("Oversampling Ratio Configuration", field_configs)

    @property
    def oversampling_ratio(self) -> int:
        return self._response_byte & 0x07

    def set_oversampling_ratio(self, ratio: int):
        # TODO: Implement the method to set the OSR field
        pass

class TLA2528OperatingModeConfiguration:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def __repr__(self):
        field_configs = [
            ("CLK_DIV", self.sampling_speed_control, 4),
            ("OSC_SEL", self.oscillator, 1),
            ("RESERVED[5:7]", 0, 3),  # Reserved bits
        ]
        return format_register("Operating Mode Configuration", field_configs)

    @property
    def oscillator(self) -> bool:
        return bool((self._response_byte & 0x10) >> 4)

    @property
    def sampling_speed_control(self) -> int:
        return self._response_byte & 0x0F

    def set_oscillator(self, oscillator: bool):
        # TODO: Implement the method to set the OSC_SEL field
        pass

    def set_sampling_speed_control(self, control: int):
        # TODO: Implement the method to set the CLK_DIV field
        pass

class TLA2528ManualChannelSelect:
    MANUAL_CH_SEL_ADDRESS = 0x11
    MANUAL_CH_ID_SIZE = 4
    MANUAL_CH_ID_OFFSET = 0
    MANUAL_CH_ID_MASK = create_bitmask(MANUAL_CH_ID_SIZE, MANUAL_CH_ID_OFFSET)
    def __init__(self, tla: 'TLA2528') -> None:
        self._tla = tla
        self._response_byte: int | None = None

    def refresh(self):
        response_bytes = self._tla.read_bytes(self.MANUAL_CH_SEL_ADDRESS)
        self._response_byte = response_bytes[0]

    @property
    def manual_channel_id(self) -> int:
        assert self._response_byte is not None
        return self._response_byte & self.MANUAL_CH_ID_MASK

    def set_manual_channel_id(self, channel_id: int):
        assert channel_id >= 0 and channel_id < 8
        assert self._response_byte is not None
        replaced = replace_bits(self._response_byte, channel_id, self.MANUAL_CH_ID_SIZE, self.MANUAL_CH_ID_OFFSET)
        self._tla.write_byte(self.MANUAL_CH_SEL_ADDRESS, replaced)

class TLA2528PinConfiguration:
    NUM_CHANNELS = 8

    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def is_channel_analog_input(self, channel: int) -> bool:
        return not bool(self._response_byte & (1 << channel))

    def configure_channel(self, channel: int, analog_input: bool):
        # TODO: Implement the method to set the PIN_CFG field
        pass

class TLA2528GpioConfiguration:
    NUM_GPIOS = 8

    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def is_gpio_input(self, gpio: int) -> bool:
        return not bool(self._response_byte & (1 << gpio))

    def configure_gpio(self, gpio: int, is_input: bool):
        # TODO: Implement the method to set the GPIO_CFG field
        pass

class TLA2528GpoDriveConfiguration:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def is_output_open_drain(self, gpo: int) -> bool:
        return not bool(self._response_byte & (1 << gpo))

    def configure_output(self, gpo: int, open_drain: bool):
        # TODO: Implement the method to set the GPO_DRIVE_CFG field
        pass

class TLA2528GpoValue:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def get_output_value(self, gpo: int) -> bool:
        return bool(self._response_byte & (1 << gpo))

    def set_output_value(self, gpo: int, value: bool):
        # TODO: Implement the method to set the GPO_VALUE field
        pass

class TLA2528GpiValue:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def get_input_value(self, gpi: int) -> bool:
        return bool(self._response_byte & (1 << gpi))

class TLA2528GeneralConfiguration:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    def __repr__(self) -> str:
        field_configs = [
            ("RST", 0, 1),
            ("CAL", 0, 1),
            ("CH_RST", self.analog_inputs_forced, 1),
            ("CNVST", self.conversion_initiated, 1),
            ("RESERVED[4:7]", 0, 4)  # Reserved bits
        ]
        return format_register("General Configuration", field_configs)

    @property
    def adc_offset_calibrated(self) -> bool:
        result = (self._response_byte & 0x02) >> 1
        return bool(result)

    @property
    def analog_inputs_forced(self) -> bool:
        result = (self._response_byte & 0x04) >> 2
        return bool(result)

    @property
    def conversion_initiated(self) -> bool:
        result = (self._response_byte & 0x08) >> 3
        return bool(result)

    def reset_device(self):
        # TODO: Implement the method to set the RST field
        pass

    def calibrate_adc_offset(self):
        # TODO: Implement the method to set the CAL field
        pass

    def force_analog_inputs(self, enable: bool):
        # TODO: Implement the method to set the CH_RST field
        pass

    def initiate_conversion(self, enable: bool):
        # TODO: Implement the method to set the CNVST field
        pass

class TLA2528SystemStatus:
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    @property
    def brown_out_reset(self) -> bool:
        result = self._response_byte & 0x01
        return bool(result)

    @property
    def crc_error_in(self) -> bool:
        result = (self._response_byte & 0x02) >> 1
        return bool(result)

    @property
    def crc_error_fuse(self) -> bool:
        result = (self._response_byte & 0x04) >> 2
        return bool(result)

    @property
    def osr_done(self) -> bool:
        result = (self._response_byte & 0x08) >> 3
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

    READ_BYTE = 0b00010000
    WRITE_BYTE = 0b00001000
    def __init__(self, i2c: I2C, device_addr: int) -> None:
        self._i2c = i2c
        self._device_addr = device_addr

    def read_bytes(self, register_address: int, bytecount: int=1) -> bytes:
        read_request = bytes([self.READ_BYTE, register_address])
        i2c.writeto(self._device_addr, read_request)
        read_bytes = i2c.readfrom(device, bytecount)
        if DEBUG_MODE:
            print(f"read req: {bytes_to_bits(read_request)}")
            print(f"read bytes: {bytes_to_bits(read_bytes)}")
        return read_bytes
    
    def write_byte(self, register_address: int, write_byte: int):
        write_request = bytes([self.WRITE_BYTE, register_address, write_byte])
        i2c.writeto(self._device_addr, write_request)
        if DEBUG_MODE:
            print(f"write req: {bytes_to_bits(write_request)}")

    def read_voltage(self) -> int:
        read_bytes = i2c.readfrom(self._device_addr, 2)
        voltage = adc_to_voltage(read_bytes, avdd=4.97)
        return voltage

    def get_system_status(self) -> TLA2528SystemStatus:
        SYSTEM_STATUS_ADDRESS = 0x00
        read = self.read_bytes(SYSTEM_STATUS_ADDRESS)
        return TLA2528SystemStatus(read)

    def get_general_configuration(self) -> TLA2528GeneralConfiguration:
        GENERAL_CFG_ADDRESS = 0x01
        read = self.read_bytes(GENERAL_CFG_ADDRESS)
        return TLA2528GeneralConfiguration(read)

    def get_oversampling_ratio_configuration(self) -> TLA2528OversamplingRatioConfiguration:
        OSR_CFG_ADDRESS = 0x3
        read = self.read_bytes(OSR_CFG_ADDRESS)
        return TLA2528OversamplingRatioConfiguration(read)

    def get_operating_mode_configuration(self) -> TLA2528OperatingModeConfiguration:
        OPMODE_CFG_ADDRESS = 0x4
        read = self.read_bytes(OPMODE_CFG_ADDRESS)
        return TLA2528OperatingModeConfiguration(read)

    def get_pin_configuration(self) -> TLA2528PinConfiguration:
        PIN_CFG_ADDRESS = 0x5
        read = self.read_bytes(PIN_CFG_ADDRESS)
        return TLA2528PinConfiguration(read)

    def get_gpio_configuration(self) -> TLA2528GpioConfiguration:
        GPIO_CFG_ADDRESS = 0x7
        read = self.read_bytes(GPIO_CFG_ADDRESS)
        return TLA2528GpioConfiguration(read)

    def get_gpo_drive_configuration(self) -> TLA2528GpoDriveConfiguration:
        GPO_DRIVE_CFG_ADDRESS = 0x9
        read = self.read_bytes(GPO_DRIVE_CFG_ADDRESS)
        return TLA2528GpoDriveConfiguration(read)

    def get_gpo_value(self) -> TLA2528GpoValue:
        GPO_VALUE_ADDRESS = 0xB
        read = self.read_bytes(GPO_VALUE_ADDRESS)
        return TLA2528GpoValue(read)

    def get_gpi_value(self) -> TLA2528GpiValue:
        GPI_VALUE_ADDRESS = 0xD
        read = self.read_bytes(GPI_VALUE_ADDRESS)
        return TLA2528GpiValue(read)
    
    def get_manual_channel_select(self) -> TLA2528ManualChannelSelect:
        manual_select = TLA2528ManualChannelSelect(self)
        manual_select.refresh()
        return manual_select


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
manual_select = chip.get_manual_channel_select()
uart = UART(1, baudrate=9600)#,tx=Pin(4),rx=Pin(5))
while True:
    """
    rxData=bytes()
    uart_data = uart.readline()
    if uart_data:
        sentence = str(uart_data).strip("b'\\r\\n")
        parsed_data = parse_nmea_sentence(sentence)
        if parsed_data:
            print(parsed_data)
        else:
            print(f"Unsupported sentence: {sentence}")
    else:
        sleep(0.1)  # If no data is available, wait for 100 ms before checking again

    """
    manual_select.set_manual_channel_id(0)
    manual_select.refresh()
    print(manual_select.manual_channel_id)
    print(f"voltage[0]: {chip.read_voltage()}")

    manual_select.set_manual_channel_id(1)
    manual_select.refresh()
    print(manual_select.manual_channel_id)
    print(f"voltage[1]: {chip.read_voltage()}")

    manual_select.set_manual_channel_id(2)
    manual_select.refresh()
    print(manual_select.manual_channel_id)
    print(f"voltage[2]: {chip.read_voltage()}")


    pin.toggle()
    sleep(0.1)
    print("----------")
