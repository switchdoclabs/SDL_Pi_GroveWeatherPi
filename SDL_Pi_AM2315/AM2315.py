#!/usr/bin/python 
# modified for GroveWeatherPi to return CRC information
# SwitchDoc Labs, 2019

# COPYRIGHT 2016 Robert Wolterman
# MIT License, see LICENSE file for details

# MODULE IMPORTS
import time

# GLOBAL VARIABLES
AM2315_I2CADDR = 0x5C
AM2315_READREG = 0x03
MAXREADATTEMPT = 3

class AM2315:
    """Base functionality for AM2315 humidity and temperature sensor. """

    def __init__(self, address=AM2315_I2CADDR, i2c=None, **kwargs):
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        self.humidity = 0
        self.temperature = 0
        self.crc = 0


    def verify_crc(self, char):
        """Returns the 16-bit CRC of sensor data"""
        crc = 0xFFFF
        for l in char:
                crc = crc ^ l
                for i in range(1,9):
                    if(crc & 0x01):
                         crc = crc >> 1
                         crc = crc ^ 0xA001
                    else:
                         crc = crc >> 1
        return crc


    def _read_data(self):
        count = 0
        tmp = None
        while count <= MAXREADATTEMPT:
            try:
                # WAKE UP
                self._device.write8(AM2315_READREG,0x00)
                time.sleep(0.09)
                # TELL THE DEVICE WE WANT 4 BYTES OF DATA
                self._device.writeList(AM2315_READREG,[0x00, 0x04])
                time.sleep(0.09)
                tmp = self._device.readList(AM2315_READREG,8)
                # IF WE HAVE DATA, LETS EXIT THIS LOOP
                if tmp != None:
                    break
            except:
                count += 1
                time.sleep(0.01)
        
        # GET THE DATA OUT OF THE LIST WE READ
        self.humidity = ((tmp[2] << 8) | tmp[3]) / 10.0
        self.temperature = (((tmp[4] & 0x7F) << 8) | tmp[5]) / 10.0
        if (tmp[4] & 0x80):
            self.temperature = -self.temperature

        self.crc = ((tmp[7] << 8) | tmp[6]) 
        # Verify CRC here
        # force CRC error with the next line
        #tmp[0] = tmp[0]+1
        t = bytearray([tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]])
        c = self.verify_crc(t)

        if self.crc != c:
            self.crc = -1

    def read_temperature(self):
        self._read_data()
        return self.temperature

    def read_humidity(self):
        self._read_data()
        return self.humidity

    def read_humidity_temperature(self):
        self._read_data()
        return (self.humidity, self.temperature)

    def read_humidity_temperature_crc(self):
        self._read_data()
        return (self.humidity, self.temperature, self.crc)

if __name__ == "__main__":
    am2315 = AM2315()
    print am2315.read_temperature()
    print am2315.read_humidity()
    print am2315.read_humidity_temperature()
    print am2315.read_humidity_temperature_crc()
    
