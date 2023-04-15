from machine import Pin, I2C
from time import sleep, sleep_ms

READ_BYTE = 0b00010000
DEBUG_MODE = True

def adc_to_voltage(adc_bytes: bytes, avdd=5.0, n_bits=12):
    # Calculate the voltage per least significant bit (LSB)
    adc_value = int.from_bytes(adc_bytes, 'big')
    print(bin(adc_value))
    adc_value = adc_value >> 4
    print(bin(adc_value))

    voltage_per_lsb = avdd / (2 ** n_bits)
    print(voltage_per_lsb)
    # Convert the 2-byte ADC value to voltage
    voltage = adc_value * voltage_per_lsb
    return voltage

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
    def __init__(self, response: bytes) -> None:
        self._response_byte = response[0]

    @property
    def manual_channel_id(self) -> int:
        return self._response_byte & 0x0F


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
        MANUAL_CH_SEL_ADDRESS = 0x11
        read = self.read_bytes(MANUAL_CH_SEL_ADDRESS)
        return TLA2528ManualChannelSelect(read)

    """
        def get_sequence_cfg(self) -> TLA2528SequenceCfg:
            SEQUENCE_CFG_ADDRESS = 0x10
            read = self.read_bytes(SEQUENCE_CFG_ADDRESS)
            return TLA2528SequenceCfg(read)

        def get_channel_sel(self) -> TLA2528ChannelSel:
            CHANNEL_SEL_ADDRESS = 0x11
            read = self.read_bytes(CHANNEL_SEL_ADDRESS)
            return TLA2528ChannelSel(read)

        def get_auto_seq_ch_sel(self) -> TLA2528AutoSeqChSel:
            AUTO_SEQ_CH_SEL_ADDRESS = 0x12
            read = self.read_bytes(AUTO_SEQ_CH_SEL_ADDRESS)
            return TLA2528AutoSeqChSel(read)
    """


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
while True:
    status = chip.get_system_status()
    print(status)
    conf = chip.get_general_configuration()
    print(conf)
    ovs = chip.get_oversampling_ratio_configuration()
    print(ovs)
    manual_select = chip.get_manual_channel_select()
    print(manual_select.manual_channel_id)
    read_bytes = i2c.readfrom(device, 2)
    print(bytes_to_bits(read_bytes))
    v = adc_to_voltage(read_bytes, avdd=4.97)
    print(f"voltage: {v}")
    print('->')
    pin.toggle()
    sleep(0.1)
