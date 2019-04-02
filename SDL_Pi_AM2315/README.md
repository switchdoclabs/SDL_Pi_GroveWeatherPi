#
# SDL_Pi_AM2315
#
# AM2315 Pure Python Library
# SwitchDoc Labs March 2019
#
#
Version 1.4:  March 30, 2019 - Added PowerSave capability and improved temperature detection<BR>
Version 1.3:  February 6, 2019 - increased MaxRetries to 10 per Sopwith - Updated to Python 3 <BR>
Version 1.2:  November 26, 2018 - Added bad data filter, now can return statistics good, bad reads, bad CRC<BR>
Version 1.1:  November 14, 2018 - Added CRC Check.  Now returns -1 in CRC on CRC Fail <BR>
 

#Introduction
This is a pure python AM2315 library to replace the tentacle_pi C based library

For the SwitchDoc Labs AM2315<BR>
https://shop.switchdoc.com/products/grove-am2315-encased-i2c-temperature-humidity-sensor-for-raspberry-pi-arduino

#Installation

Requires installation of Adafruit_Python_GPIO

```
sudo apt-get update
sudo apt-get install build-essential python-pip python-dev python-smbus git
git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
cd Adafruit_Python_GPIO
sudo python setup.py install
```


Place files in program directory

# testing

```
import AM2315 
sens = AM2315.AM2315()
print sens.read_temperature()
print sens.read_humidity()
print sens.read_humidity_temperature()
print sens.read_humidity_temperature_crc()
```
