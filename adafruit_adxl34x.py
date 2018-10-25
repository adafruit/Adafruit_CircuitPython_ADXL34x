# The MIT License (MIT)
#
# Copyright (c) 2018 Bryan Siepert for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_adxl34x`
====================================================

A driver for the ADXL34x 3-axis accelerometer family

* Author(s): Bryan Siepert

Based on drivers by K. Townsend and Tony DiCola

Implementation Notes
--------------------

**Hardware:**
https://www.adafruit.com/product/1231

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from adafruit_bus_device import i2c_device
try:
    from struct import unpack
except ImportError:
    from ustruct import unpack
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x.git"
#pylint: disable=bad-whitespace
_ADXL345_DEFAULT_ADDRESS         = const(0x53) # Assumes ALT address pin low

# Conversion factors
_ADXL345_MG2G_MULTIPLIER = 0.004 # 4mg per lsb
_STANDARD_GRAVITY = 9.80665 # earth standard gravity

_ADXL345_REG_DEVID               = const(0x00) # Device ID
_ADXL345_REG_THRESH_TAP          = const(0x1D) # Tap threshold
_ADXL345_REG_OFSX                = const(0x1E) # X-axis offset
_ADXL345_REG_OFSY                = const(0x1F) # Y-axis offset
_ADXL345_REG_OFSZ                = const(0x20) # Z-axis offset
_ADXL345_REG_DUR                 = const(0x21) # Tap duration
_ADXL345_REG_LATENT              = const(0x22) # Tap latency
_ADXL345_REG_WINDOW              = const(0x23) # Tap window
_ADXL345_REG_THRESH_ACT          = const(0x24) # Activity threshold
_ADXL345_REG_THRESH_INACT        = const(0x25) # Inactivity threshold
_ADXL345_REG_TIME_INACT          = const(0x26) # Inactivity time
_ADXL345_REG_ACT_INACT_CTL       = const(0x27) # Axis enable control for [in]activity detection
_ADXL345_REG_THRESH_FF           = const(0x28) # Free-fall threshold
_ADXL345_REG_TIME_FF             = const(0x29) # Free-fall time
_ADXL345_REG_TAP_AXES            = const(0x2A) # Axis control for single/double tap
_ADXL345_REG_ACT_TAP_STATUS      = const(0x2B) # Source for single/double tap
_ADXL345_REG_BW_RATE             = const(0x2C) # Data rate and power mode control
_ADXL345_REG_POWER_CTL           = const(0x2D) # Power-saving features control
_ADXL345_REG_INT_ENABLE          = const(0x2E) # Interrupt enable control
_ADXL345_REG_INT_MAP             = const(0x2F) # Interrupt mapping control
_ADXL345_REG_INT_SOURCE          = const(0x30) # Source of interrupts
_ADXL345_REG_DATA_FORMAT         = const(0x31) # Data format control
_ADXL345_REG_DATAX0              = const(0x32) # X-axis data 0
_ADXL345_REG_DATAX1              = const(0x33) # X-axis data 1
_ADXL345_REG_DATAY0              = const(0x34) # Y-axis data 0
_ADXL345_REG_DATAY1              = const(0x35) # Y-axis data 1
_ADXL345_REG_DATAZ0              = const(0x36) # Z-axis data 0
_ADXL345_REG_DATAZ1              = const(0x37) # Z-axis data 1
_ADXL345_REG_FIFO_CTL            = const(0x38) # FIFO control
_ADXL345_REG_FIFO_STATUS         = const(0x39) # FIFO status

class DataRate: #pylint: disable=too-few-public-methods
    """An enum-like class representing the possible data rates.
    Possible values are

    - ``DataRate.RATE_3200_HZ``
    - ``DataRate.RATE_1600_HZ``
    - ``DataRate.RATE_800_HZ``
    - ``DataRate.RATE_400_HZ``
    - ``DataRate.RATE_200_HZ``
    - ``DataRate.RATE_100_HZ``
    - ``DataRate.RATE_50_HZ``
    - ``DataRate.RATE_25_HZ``
    - ``DataRate.RATE_12_5_HZ``
    - ``DataRate.RATE_6_25HZ``
    - ``DataRate.RATE_3_13_HZ``
    - ``DataRate.RATE_1_56_HZ``
    - ``DataRate.RATE_0_78_HZ``
    - ``DataRate.RATE_0_39_HZ``
    - ``DataRate.RATE_0_20_HZ``
    - ``DataRate.RATE_0_10_HZ``

    """
    RATE_3200_HZ    = const(0b1111) # 1600Hz Bandwidth   140mA IDD
    RATE_1600_HZ    = const(0b1110) #  800Hz Bandwidth    90mA IDD
    RATE_800_HZ     = const(0b1101) #  400Hz Bandwidth   140mA IDD
    RATE_400_HZ     = const(0b1100) #  200Hz Bandwidth   140mA IDD
    RATE_200_HZ     = const(0b1011) #  100Hz Bandwidth   140mA IDD
    RATE_100_HZ     = const(0b1010) #   50Hz Bandwidth   140mA IDD
    RATE_50_HZ      = const(0b1001) #   25Hz Bandwidth    90mA IDD
    RATE_25_HZ      = const(0b1000) # 12.5Hz Bandwidth    60mA IDD
    RATE_12_5_HZ    = const(0b0111) # 6.25Hz Bandwidth    50mA IDD
    RATE_6_25HZ     = const(0b0110) # 3.13Hz Bandwidth    45mA IDD
    RATE_3_13_HZ    = const(0b0101) # 1.56Hz Bandwidth    40mA IDD
    RATE_1_56_HZ    = const(0b0100) # 0.78Hz Bandwidth    34mA IDD
    RATE_0_78_HZ    = const(0b0011) # 0.39Hz Bandwidth    23mA IDD
    RATE_0_39_HZ    = const(0b0010) # 0.20Hz Bandwidth    23mA IDD
    RATE_0_20_HZ    = const(0b0001) # 0.10Hz Bandwidth    23mA IDD
    RATE_0_10_HZ    = const(0b0000) # 0.05Hz Bandwidth    23mA IDD (default value)


class Range: #pylint: disable=too-few-public-methods
    """An enum-like class representing the possible measurement ranges in +/- G.

    Possible values are

    - ``Range.RANGE_16_G``
    - ``Range.RANGE_8_G``
    - ``Range.RANGE_4_G``
    - ``Range.RANGE_2_G``

    """
    RANGE_16_G  = const(0b11)   # +/- 16g
    RANGE_8_G   = const(0b10)   # +/- 8g
    RANGE_4_G   = const(0b01)   # +/- 4g
    RANGE_2_G   = const(0b00)   # +/- 2g (default value)

# pylint: enable=bad-whitespace
class ADXL345:
    """Driver for the ADXL345 3 axis accelerometer

        :param ~busio.I2C i2c_bus: The I2C bus the ADXL345 is connected to.
        :param address: The I2C device address for the sensor. Default is ``0x53``.

    """
    def __init__(self, i2c, address=_ADXL345_DEFAULT_ADDRESS):

        self._i2c = i2c_device.I2CDevice(i2c, address)
        self._buffer = bytearray(6)
        # set the 'measure' bit in to enable measurement
        self._write_register_byte(_ADXL345_REG_POWER_CTL, 0x08)

    @property
    def acceleration(self):
        """The x, y, z acceleration values returned in a 3-tuple in m / s ^ 2."""
        x, y, z = unpack('<hhh', self._read_register(_ADXL345_REG_DATAX0, 6))
        x = x * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
        y = y * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
        z = z * _ADXL345_MG2G_MULTIPLIER * _STANDARD_GRAVITY
        return (x, y, z)

    @property
    def data_rate(self):
        """The data rate of the sensor."""
        rate_register = unpack("<b", self._read_register(_ADXL345_REG_BW_RATE, 1))[0]
        return rate_register & 0x0F

    @data_rate.setter
    def data_rate(self, val):
        self._write_register_byte(_ADXL345_REG_BW_RATE, val)

    @property
    def range(self):
        """The measurement range of the sensor."""
        range_register = unpack("<b", self._read_register(_ADXL345_REG_DATA_FORMAT, 1))[0]
        return range_register & 0x03

    @range.setter
    def range(self, val):
        # read the current value of the data format register
        format_register = unpack("<b", self._read_register(_ADXL345_REG_DATA_FORMAT, 1))[0]

        # clear the bottom 4 bits and update the data rate
        format_register &= ~0x0F
        format_register |= val

        # Make sure that the FULL-RES bit is enabled for range scaling
        format_register |= 0x08

        # write the updated values
        self._write_register_byte(_ADXL345_REG_DATA_FORMAT, format_register)

    def _read_register(self, register, length):
        self._buffer[0] = register & 0xFF
        with self._i2c as i2c:
            i2c.write(self._buffer, start=0, end=1)
            i2c.readinto(self._buffer, start=0, end=length)
            return self._buffer

    def _write_register_byte(self, register, value):
        self._buffer[0] = register & 0xFF
        self._buffer[1] = value & 0xFF
        with self._i2c as i2c:
            i2c.write(self._buffer, start=0, end=2)
