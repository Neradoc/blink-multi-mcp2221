# SPDX-FileCopyrightText: 2021 Melissa LeBlanc-Williams for Adafruit Industries
# SPDX-FileCopyrightText: 2023 Neradoc https://neradoc.me
#
# SPDX-License-Identifier: MIT
"""Chip Definition for MCP2221"""

import atexit
import hid
import os
import time
from .pin import Pin
from .i2c import I2C

# Here if you need it
MCP2221_HID_DELAY = float(os.environ.get("BLINKA_MCP2221_HID_DELAY", 0))
# Use to set delay between reset and device reopen. if negative, don't reset at all
MCP2221_RESET_DELAY = float(os.environ.get("BLINKA_MCP2221_RESET_DELAY", 0.5))

# from the C driver
# http://ww1.microchip.com/downloads/en/DeviceDoc/mcp2221_0_1.tar.gz
# others (???) determined during driver developement
RESP_ERR_NOERR = 0x00
RESP_ADDR_NACK = 0x25
RESP_READ_ERR = 0x7F
RESP_READ_COMPL = 0x55
RESP_READ_PARTIAL = 0x54  # ???
RESP_I2C_IDLE = 0x00
RESP_I2C_START_TOUT = 0x12
RESP_I2C_RSTART_TOUT = 0x17
RESP_I2C_WRADDRL_TOUT = 0x23
RESP_I2C_WRADDRL_WSEND = 0x21
RESP_I2C_WRADDRL_NACK = 0x25
RESP_I2C_WRDATA_TOUT = 0x44
RESP_I2C_RDDATA_TOUT = 0x52
RESP_I2C_STOP_TOUT = 0x62

RESP_I2C_MOREDATA = 0x43  # ???
RESP_I2C_PARTIALDATA = 0x41  # ???
RESP_I2C_WRITINGNOSTOP = 0x45  # ???

MCP2221_RETRY_MAX = 50
MCP2221_MAX_I2C_DATA_LEN = 60
MASK_ADDR_NACK = 0x40

STATUS_COMMAND = b"\x10"
SRAM_COMMAND = b"\x61"

FLASH_SETTINGS_SUBCODE = b"\x00"
SERIAL_NUMBER_SUBCODE = b"\x04"


class Board:
    def __init__(self, mcp2221):
        self._mcp = mcp2221

        # create pin instances for each pin
        self.G0 = Pin(mcp2221, 0)
        self.G1 = Pin(mcp2221, 1)
        self.G2 = Pin(mcp2221, 2)
        self.G3 = Pin(mcp2221, 3)

        self.SCL = Pin(mcp2221)
        self.SDA = Pin(mcp2221)
        self._i2c = None

        self.board_id = "mcp2221"

    def I2C(self):
        if self._i2c is None:
            self._i2c = I2C(self._mcp)
        return self._i2c


class MCP2221:
    """MCP2221 Device Class Definition"""

    VID = 0x04D8
    PID = 0x00DD

    GP_GPIO = 0b000
    GP_DEDICATED = 0b001
    GP_ALT0 = 0b010
    GP_ALT1 = 0b011
    GP_ALT2 = 0b100

    def __init__(self,
            hid_delay=MCP2221_HID_DELAY,
            reset_delay=MCP2221_RESET_DELAY
            ):
        self.serial_number = None
        self.hid_delay = hid_delay
        self.reset_delay = reset_delay
        self._hid = hid.device()
        self._hid.open(MCP2221.VID, MCP2221.PID)
        # make sure the device gets closed before exit
        atexit.register(self.close)
        if self.reset_delay >= 0:
            self._reset()
        self._gp_config = [0x07] * 4  # "don't care" initial value
        for pin in range(4):
            self.gp_set_mode(pin, self.GP_GPIO)  # set to GPIO mode
            self.gpio_set_direction(pin, 1)  # set to INPUT
        self.board = Board(self)
        self.serial_number = self._get_serial_number()

    def close(self):
        """Close the hid device. Does nothing if the device is not open."""
        self._hid.close()

    def __del__(self):
        # try to close the device before destroying the instance
        self.deinit()

    def deinit(self):
        # manually deinit for sure
        self.close()

    def _hid_xfer(self, report, response=True):
        """Perform HID Transfer"""
        # first byte is report ID, which =0 for MCP2221
        # remaing bytes = 64 byte report data
        # https://github.com/libusb/hidapi/blob/083223e77952e1ef57e6b77796536a3359c1b2a3/hidapi/hidapi.h#L185
        self._hid.write(b"\0" + report + b"\0" * (64 - len(report)))
        time.sleep(self.hid_delay)
        if response:
            # return is 64 byte response report
            return self._hid.read(64)
        return None

    def _read_flash(self, sub_code):
        self._hid.write(b'\xB0' + sub_code + (b'\x00' * 62))
        info = self._hid.read(64)
        if info[1] == 0x01:
            print('Command not supported')
        return info

    def _get_serial_number(self):
        try:
            # enable serial number enumeration with enable_serial()
            return self._hid.get_serial_number_string()
        except OSError:
            pass
        data = self._read_flash(SERIAL_NUMBER_SUBCODE)
        if data[3] != 3:
            return False
        length = data[2]
        return ''.join(chr(x) for x in data[4:4+length] if x > 0)

    def _write_flash(self, data):
        self._hid.write(b'\xB1' + data)
        info = self._hid.read(64)
        assert info[0] == 0xB1
        if info[1] == 0x02:
            raise FlashError('Command not supported')
        if info[1] == 0x03:
            raise FlashError('Command not allowed')

    def write_product(self, name):
        name = name.encode("utf8")
        if len(name) > 29:
            raise ValueError('Too long')
        ps = b''.join([c + b'\x00' for c in name])
        data = b'\x03' + bytes([2 * len(name) + 2]) + b'\x03' + ps
        return self._write_flash(data)

    def write_manufacturer(self, name):
        name = name.encode("utf8")
        if len(name) > 29:
            raise ValueError('Too long')
        ps = b''.join([c + b'\x00' for c in name])
        data = b'\x02' + bytes([2 * len(name) + 2]) + b'\x03' + ps
        return self._write_flash(data)

    def enable_serial(self):
        # read settings
        data = self._read_flash(FLASH_SETTINGS_SUBCODE)
        # set bytes 4 bit 7 to 1
        data[4] |= 1 << 7
        # write back all bytes
        self._write_flash(b"\x00" + bytes(data[4:24]))

    # ----------------------------------------------------------------
    # MISC
    # ----------------------------------------------------------------
    def gp_get_mode(self, pin):
        """Get Current Pin Mode"""
        return self._hid_xfer(SRAM_COMMAND)[22 + pin] & 0x07

    def gp_set_mode(self, pin, mode):
        """Set Current Pin Mode"""
        # already set to that mode?
        mode &= 0x07
        if mode == (self._gp_config[pin] & 0x07):
            return
        # update GP mode for pin
        self._gp_config[pin] = mode
        # empty report, this is safe since 0's = no change
        report = bytearray(b"\x60" + b"\x00" * 63)
        # set the alter GP flag byte
        report[7] = 0xFF
        # add GP setttings
        report[8] = self._gp_config[0]
        report[9] = self._gp_config[1]
        report[10] = self._gp_config[2]
        report[11] = self._gp_config[3]
        # and make it so
        self._hid_xfer(report)

    def _pretty_report(self, register):
        report = self._hid_xfer(register)
        print("     0  1  2  3  4  5  6  7  8  9")
        index = 0
        for row in range(7):
            print("{} : ".format(row), end="")
            for _ in range(10):
                print("{:02x} ".format(report[index]), end="")
                index += 1
                if index > 63:
                    break
            print()

    def _status_dump(self):
        self._pretty_report(STATUS_COMMAND)

    def _sram_dump(self):
        self._pretty_report(SRAM_COMMAND)

    def _reset(self):
        self._hid_xfer(b"\x70\xAB\xCD\xEF", response=False)
        self._hid.close()
        time.sleep(self.reset_delay)
        start = time.monotonic()
        while time.monotonic() - start < 5:
            try:
                self._hid.open(MCP2221.VID, MCP2221.PID)
            except OSError:
                # try again
                time.sleep(0.1)
                continue
            if self.serial_number is None:
                return
            elif self._get_serial_number() == self.serial_number:
                return
            else:
                self._hid.close()
        raise OSError("open failed")

    # ----------------------------------------------------------------
    # GPIO
    # ----------------------------------------------------------------
    def gpio_set_direction(self, pin, mode):
        """Set Current GPIO Pin Direction"""
        if mode:
            # set bit 3 for INPUT
            self._gp_config[pin] |= 1 << 3
        else:
            # clear bit 3 for OUTPUT
            self._gp_config[pin] &= ~(1 << 3)
        report = bytearray(b"\x50" + b"\x00" * 63)  # empty set GPIO report
        offset = 4 * (pin + 1)
        report[offset] = 0x01  # set pin direction
        report[offset + 1] = mode  # to this
        self._hid_xfer(report)

    def gpio_set_pin(self, pin, value):
        """Set Current GPIO Pin Value"""
        if value:
            # set bit 4
            self._gp_config[pin] |= 1 << 4
        else:
            # clear bit 4
            self._gp_config[pin] &= ~(1 << 4)
        report = bytearray(b"\x50" + b"\x00" * 63)  # empty set GPIO report
        offset = 2 + 4 * pin
        report[offset] = 0x01  # set pin value
        report[offset + 1] = value  # to this
        self._hid_xfer(report)

    def gpio_get_pin(self, pin):
        """Get Current GPIO Pin Value"""
        resp = self._hid_xfer(b"\x51")
        offset = 2 + 2 * pin
        if resp[offset] == 0xEE:
            raise RuntimeError("Pin is not set for GPIO operation.")
        return resp[offset]

    # ----------------------------------------------------------------
    # I2C
    # ----------------------------------------------------------------
    def _i2c_status(self):
        resp = self._hid_xfer(b"\x10")
        if resp[1] != 0:
            raise RuntimeError("Couldn't get I2C status")
        return resp

    def _i2c_state(self):
        return self._i2c_status()[8]

    def _i2c_cancel(self):
        resp = self._hid_xfer(b"\x10\x00\x10")
        if resp[1] != 0x00:
            raise RuntimeError("Couldn't cancel I2C")
        if resp[2] == 0x10:
            # bus release will need "a few hundred microseconds"
            time.sleep(0.001)

    # pylint: disable=too-many-arguments,too-many-branches
    def _i2c_write(self, cmd, address, buffer, start=0, end=None):
        if self._i2c_state() != 0x00:
            self._i2c_cancel()

        end = end if end else len(buffer)
        length = end - start
        retries = 0

        while (end - start) > 0 or not buffer:
            chunk = min(end - start, MCP2221_MAX_I2C_DATA_LEN)
            # write out current chunk
            resp = self._hid_xfer(
                bytes([cmd, length & 0xFF, (length >> 8) & 0xFF, address << 1])
                + buffer[start : (start + chunk)]
            )
            # check for success
            if resp[1] != 0x00:
                if resp[2] in (
                    RESP_I2C_START_TOUT,
                    RESP_I2C_WRADDRL_TOUT,
                    RESP_I2C_WRADDRL_NACK,
                    RESP_I2C_WRDATA_TOUT,
                    RESP_I2C_STOP_TOUT,
                ):
                    raise RuntimeError("Unrecoverable I2C state failure")
                retries += 1
                if retries >= MCP2221_RETRY_MAX:
                    raise RuntimeError("I2C write error, max retries reached.")
                time.sleep(0.001)
                continue  # try again
            # yay chunk sent!
            while self._i2c_state() == RESP_I2C_PARTIALDATA:
                time.sleep(0.001)
            if not buffer:
                break
            start += chunk
            retries = 0

        # check status in another loop
        for _ in range(MCP2221_RETRY_MAX):
            status = self._i2c_status()
            if status[20] & MASK_ADDR_NACK:
                raise RuntimeError("I2C slave address was NACK'd")
            usb_cmd_status = status[8]
            if usb_cmd_status == 0:
                break
            if usb_cmd_status == RESP_I2C_WRITINGNOSTOP and cmd == 0x94:
                break  # this is OK too!
            if usb_cmd_status in (
                RESP_I2C_START_TOUT,
                RESP_I2C_WRADDRL_TOUT,
                RESP_I2C_WRADDRL_NACK,
                RESP_I2C_WRDATA_TOUT,
                RESP_I2C_STOP_TOUT,
            ):
                raise RuntimeError("Unrecoverable I2C state failure")
            time.sleep(0.001)
        else:
            raise RuntimeError("I2C write error: max retries reached.")
        # whew success!

    def _i2c_read(self, cmd, address, buffer, start=0, end=None):
        if self._i2c_state() not in (RESP_I2C_WRITINGNOSTOP, 0):
            self._i2c_cancel()

        end = end if end else len(buffer)
        length = end - start

        # tell it we want to read
        resp = self._hid_xfer(
            bytes([cmd, length & 0xFF, (length >> 8) & 0xFF, (address << 1) | 0x01])
        )

        # check for success
        if resp[1] != 0x00:
            raise RuntimeError("Unrecoverable I2C read failure")

        # and now the read part
        while (end - start) > 0:
            for _ in range(MCP2221_RETRY_MAX):
                # the actual read
                resp = self._hid_xfer(b"\x40")
                # check for success
                if resp[1] == RESP_I2C_PARTIALDATA:
                    time.sleep(0.001)
                    continue
                if resp[1] != 0x00:
                    raise RuntimeError("Unrecoverable I2C read failure")
                if resp[2] == RESP_ADDR_NACK:
                    raise RuntimeError("I2C NACK")
                if resp[3] == 0x00 and resp[2] == 0x00:
                    break
                if resp[3] == RESP_READ_ERR:
                    time.sleep(0.001)
                    continue
                if resp[2] in (RESP_READ_COMPL, RESP_READ_PARTIAL):
                    break
            else:
                raise RuntimeError("I2C read error: max retries reached.")

            # move data into buffer
            chunk = min(end - start, 60)
            for i, k in enumerate(range(start, start + chunk)):
                buffer[k] = resp[4 + i]
            start += chunk

    # pylint: enable=too-many-arguments

    def _i2c_configure(self, baudrate=100000):
        """Configure I2C"""
        self._hid_xfer(
            bytes(
                [
                    0x10,  # set parameters
                    0x00,  # don't care
                    0x00,  # no effect
                    0x20,  # next byte is clock divider
                    12000000 // baudrate - 3,
                ]
            )
        )

    def i2c_writeto(self, address, buffer, *, start=0, end=None):
        """Write data from the buffer to an address"""
        self._i2c_write(0x90, address, buffer, start, end)

    def i2c_readfrom_into(self, address, buffer, *, start=0, end=None):
        """Read data from an address and into the buffer"""
        self._i2c_read(0x91, address, buffer, start, end)

    def i2c_writeto_then_readfrom(
        self,
        address,
        out_buffer,
        in_buffer,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None,
    ):
        """Write data from buffer_out to an address and then
        read data from an address and into buffer_in
        """
        self._i2c_write(0x94, address, out_buffer, out_start, out_end)
        self._i2c_read(0x93, address, in_buffer, in_start, in_end)

    def i2c_scan(self, *, start=0, end=0x79):
        """Perform an I2C Device Scan"""
        found = []
        for addr in range(start, end + 1):
            # try a write
            try:
                self.i2c_writeto(addr, b"\x00")
            except RuntimeError:  # no reply!
                continue
            # store if success
            found.append(addr)
        return found

    # ----------------------------------------------------------------
    # ADC
    # ----------------------------------------------------------------
    def adc_configure(self, vref=0):
        """Configure the Analog-to-Digital Converter"""
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[5] = 1 << 7 | (vref & 0b111)
        self._hid_xfer(report)

    def adc_read(self, pin):
        """Read from the Analog-to-Digital Converter"""
        resp = self._hid_xfer(b"\x10")
        return resp[49 + 2 * pin] << 8 | resp[48 + 2 * pin]

    # ----------------------------------------------------------------
    # DAC
    # ----------------------------------------------------------------
    def dac_configure(self, vref=0):
        """Configure the Digital-to-Analog Converter"""
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[3] = 1 << 7 | (vref & 0b111)
        self._hid_xfer(report)

    # pylint: disable=unused-argument
    def dac_write(self, pin, value):
        """Write to the Digital-to-Analog Converter"""
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[4] = 1 << 7 | (value & 0b11111)
        self._hid_xfer(report)

    # pylint: enable=unused-argument

