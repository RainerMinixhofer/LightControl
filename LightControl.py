#!/usr/bin/python3
#pylint: disable = C0103
"""
Code taken from
https://github.com/adafruit/Adafruit_Python_PCA9685/blob/master/Adafruit_PCA9685/PCA9685.py
and modified/extended for better usability.
Test included
"""
from __future__ import division
import logging
import time
import math
from smbus import SMBus #pylint: disable=E0401
import RPi.GPIO as GPIO

# Registers/etc:
# I2C address of PCA9685 on I2C bus 1
PCA9685_ADDRESS    = 0x58
# Mode Registers 1 and 2
MODE1              = 0x00
MODE2              = 0x01
# I2C-bus subadresses 1-3
SUBADR1            = 0x02
SUBADR2            = 0x03
SUBADR3            = 0x04
# Prescaler for PWM output frequency
PRESCALE           = 0xFE
# LED0 output and brightness control bytes 0-3
LED0_ON_L          = 0x06
LED0_ON_H          = 0x07
LED0_OFF_L         = 0x08
LED0_OFF_H         = 0x09
# Load all the LEDn_ON registers bytes 0-1
ALL_LED_ON_L       = 0xFA
ALL_LED_ON_H       = 0xFB
# Load all the LEDn_OFF registers bytes 0-1
ALL_LED_OFF_L      = 0xFC
ALL_LED_OFF_H      = 0xFD

# Bit positions for Mode register 1
RESTART            = 7
EXTCLK             = 6
AI                 = 5
SLEEP              = 4
SUB1               = 3
SUB2               = 2
SUB3               = 1
ALLCALL            = 0
# Bit positions for Mode register 2
INVRT              = 4
OCH                = 3
OUTDRV             = 2
OUTNE1             = 1
OUTNE0             = 0


logger = logging.getLogger(__name__)


class PCA9685:
    """PCA9685 PWM LED/servo controller."""

    def __init__(self, address=PCA9685_ADDRESS, i2c=None, autoincrement=False, totempole=True, invertoutputs=False, outnemode=0):
        """Initialize the PCA9685."""
        # Setup I2C interface for the device.
        if i2c is None:
            i2c = SMBus(1)
        self._device = i2c
        self._address = address
        self.set_all_pwm(0, 0)
        self._device.write_byte_data(self._address, MODE2, OUTDRV)
        self._device.write_byte_data(self._address, MODE1, ALLCALL)
        time.sleep(0.005)  # wait for oscillator
        self.sleep = False # wake up (reset sleep)
        time.sleep(0.005)  # wait for oscillator
        self.autoincrement = autoincrement
        self.outputdrivermode = totempole # If totempole True set LEDn outputs to totem pole structure, if False to open-drain
        self.outputinverted = invertoutputs # If invertoutputs is True then LEDn outputs are inverted
        self.outputnemode = outnemode

    def read_register_byte(self, reg):
        return self._device.read_byte_data(self._address, reg)

    def write_register_byte(self, reg, byte):
        self._device.write_byte_data(self._address, reg, byte)

    def read_register_bit(self, reg, bit):
        return (self._device.read_byte_data(self._address, reg) & (1 << bit)) == (1 << bit)

    def set_register_bit(self, reg, bit):
        value = self._device.read_byte_data(self._address, reg) | (1 << bit)
        self._device.write_byte_data(self._address, reg, value)

    def clear_register_bit(self, reg, bit):
        value = self._device.read_byte_data(self._address, reg) & ~(1 << bit)
        self._device.write_byte_data(self._address, reg, value)

    def write_register_bit(self, reg, bit, bool):
        if bool:
            self.set_register_bit(reg, bit)
        else:
            self.clear_register_bit(reg, bit)

    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency to the provided value in hertz."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        logger.debug('Setting PWM frequency to %s Hz', freq_hz)
        logger.debug('Estimated pre-scale: %s', prescaleval)
        prescale = int(math.floor(prescaleval + 0.5))
        logger.debug('Final pre-scale: %s', prescale)
        oldmode = self._device.read_byte_data(self._address, MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self._device.write_byte_data(self._address, MODE1, newmode)  # go to sleep
        self._device.write_byte_data(self._address, PRESCALE, prescale)
        self._device.write_byte_data(self._address, MODE1, oldmode)
        time.sleep(0.005)
        self._device.write_byte_data(self._address, MODE1, oldmode | 0x80)

    #Low level function to set the on and off times of the LED<channel> within
    #the duty cycle of 100% absolute period of duty cycle is set by set_pwm_freq
    #on and off are 12-bit values in the interval of 0 to 4095=0x0FFF

    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel."""
        self._device.write_byte_data(self._address, LED0_ON_L+4*channel, on & 0xFF)
        self._device.write_byte_data(self._address, LED0_ON_H+4*channel, on >> 8)
        self._device.write_byte_data(self._address, LED0_OFF_L+4*channel, off & 0xFF)
        self._device.write_byte_data(self._address, LED0_OFF_H+4*channel, off >> 8)

    #Like set_pwm but set all channels with same on and off settings

    def set_all_pwm(self, on, off):
        """Sets all PWM channels."""
        self._device.write_byte_data(self._address, ALL_LED_ON_L, on & 0xFF)
        self._device.write_byte_data(self._address, ALL_LED_ON_H, on >> 8)
        self._device.write_byte_data(self._address, ALL_LED_OFF_L, off & 0xFF)
        self._device.write_byte_data(self._address, ALL_LED_OFF_H, off >> 8)

    @property
    def autoincrement(self):
        return self.read_register_bit(MODE1, AI) == 1
    @autoincrement.setter
    def autoincrement(self, bool):
        self.write_register_bit(MODE1, AI, bool)

    @property
    def sleep(self):
        return self.read_register_bit(MODE1, SLEEP) == 1
    @sleep.setter
    def sleep(self, bool):
        self.write_register_bit(MODE1, SLEEP, bool)

    @property
    def outputinverted(self):
        return self.read_register_bit(MODE2, INVRT) == 1
    @outputinverted.setter
    def outputinverted(self, bool):
        self.write_register_bit(MODE2, INVRT, bool)

    @property
    def outputchangemode(self):
        return self.read_register_bit(MODE2, OCH) == 1
    @outputchangemode.setter
    def outputinverted(self, bool):
        self.write_register_bit(MODE2, OCH, bool)

    @property
    def outputdrivermode(self):
        return self.read_register_bit(MODE2, OUTDRV) == 1
    @outputdrivermode.setter
    def outputdrivermode(self, bool):
        self.write_register_bit(MODE2, OUTDRV, bool)

    @property
    def outputnemode(self):
        result = 0
        if self.read_register_bit(MODE2, OUTNE1) == 1:
            result += 1 << OUTNE1
        if self.read_register_bit(MODE2, OUTNE0) == 1:
            result += 1 << OUTNE0
        return result
    @outputnemode.setter
    def outputnemode(self, mode):
        self.write_register_bit(MODE2, OUTNE1, mode & 0x02)
        self.write_register_bit(MODE2, OUTNE0, mode & 0x01)

#Set pin numbering scheme to GPIO numbering
GPIO.setmode(GPIO.BCM)
#Switch GPIO4 to output
GPIO.setup(4, GPIO.OUT)
#Enable outputs (set ~OE to low)
GPIO.output(4, GPIO.LOW)

#Initialize class for PCA9685
#Set output driver mode to open-drain
#Invert outputs since no external driver is used
ctrl = PCA9685(totempole=False, invertoutputs=True, outnemode=2)
print("Output Driver Mode", "Totem-Pole" if ctrl.outputdrivermode else "Open-Drain")
print("Output Inverted?:",ctrl.outputinverted)
print("MODE2 Register",bin(ctrl.read_register_byte(MODE2)))
#Set PWM Frequency to 200 Hz
ctrl.set_pwm_freq(200)

#Switch all LEDs off
ctrl.set_all_pwm(4096, 0)

#Ramp up intensity of all LEDs
for i in reversed(range(4096)):
    ctrl.set_all_pwm(0, i)
    time.sleep(0.001)

ctrl.set_all_pwm(4096, 0)

#Ramp only red channels
for j in range(3):
    for i in reversed(range(4096)):
        ctrl.set_pwm(0+j, 0, i)
        ctrl.set_pwm(3+j, 0, i)
        ctrl.set_pwm(6+j, 0, i)
        ctrl.set_pwm(9+j, 0, i)
        time.sleep(0.001)
    ctrl.set_all_pwm(4096, 0)
    time.sleep(1.0)

ctrl.sleep = True

#for i in range(4):
#    for rgb in range(3):
#        ctrl.set_pwm(i*3+rgb, i*4095//4, (i*4095//4 + 4095//2) % 4096)

#Disable outputs (set ~OE to high)
GPIO.output(4, GPIO.HIGH)
