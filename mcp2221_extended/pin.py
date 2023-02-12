# SPDX-FileCopyrightText: 2021 Melissa LeBlanc-Williams for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""MCP2221 pin names"""

class Pin:
    """
    A basic Pin class for use with MCP2221.
    The pins don't have internal pull resistors.
    """

    # pin modes
    OUT = 0
    IN = 1
    ADC = 2
    DAC = 3
    # pin values
    LOW = 0
    HIGH = 1

    def __init__(self, mcp2221, pin_id=None):
        self.mcp = mcp2221
        self.id = pin_id
        self._mode = None

    def init(self, mode=IN, pull=None):
        """Initialize the Pin"""
        if self.id is None:
            raise RuntimeError("Can not init a None type pin.")
        if pull is not None:
            raise NotImplementedError("Internal pullups and pulldowns not supported")
        if mode in (Pin.IN, Pin.OUT):
            # All pins can do GPIO
            self.mcp.gp_set_mode(self.id, self.mcp.GP_GPIO)
            self.mcp.gpio_set_direction(self.id, mode)
        elif mode == Pin.ADC:
            # ADC only available on these pins
            if self.id not in (1, 2, 3):
                raise ValueError("Pin does not have ADC capabilities")
            self.mcp.gp_set_mode(self.id, self.mcp.GP_ALT0)
            self.mcp.adc_configure()
        elif mode == Pin.DAC:
            # DAC only available on these pins
            if self.id not in (2, 3):
                raise ValueError("Pin does not have DAC capabilities")
            self.mcp.gp_set_mode(self.id, self.mcp.GP_ALT1)
            self.mcp.dac_configure()
        else:
            raise ValueError("Incorrect pin mode: {}".format(mode))
        self._mode = mode

    def value(self, val=None):
        """Set or return the Pin Value"""
        # Digital In / Out
        if self._mode in (Pin.IN, Pin.OUT):
            # digital read
            if val is None:
                return self.mcp.gpio_get_pin(self.id)
            # digital write
            if val in (Pin.LOW, Pin.HIGH):
                self.mcp.gpio_set_pin(self.id, val)
                return None
            # nope
            raise ValueError("Invalid value for pin.")
        # Analog In
        if self._mode == Pin.ADC:
            if val is None:
                # MCP2221 ADC is 10 bit, scale to 16 bit per CP API
                return self.mcp.adc_read(self.id) * 64
            # read only
            raise AttributeError("'AnalogIn' object has no attribute 'value'")
        # Analog Out
        if self._mode == Pin.DAC:
            if val is None:
                # write only
                raise AttributeError("unreadable attribute")
            # scale 16 bit value to MCP2221 5 bit DAC (yes 5 bit)
            self.mcp.dac_write(self.id, val // 2048)
            return None
        raise RuntimeError(
            "No action for mode {} with value {}".format(self._mode, val)
        )
