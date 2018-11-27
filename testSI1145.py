 
#!/usr/bin/python

# Author: Joe Gutting
# With use of Adafruit SI1145 library for Arduino, Adafruit_GPIO.I2C & BMP Library by Tony DiCola
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

# Can enable debug output by uncommenting:
#import logging
#logging.basicConfig(level=logging.DEBUG)
import sys

sys.path.append('./SDL_Pi_TCA9545')
sys.path.append('./SDL_Pi_SI1145')

import time
import SDL_Pi_SI1145 
import SI1145Lux


import SDL_Pi_TCA9545




################
# TCA9545 I2C Mux 

#/*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
TCA9545_ADDRESS =                         (0x73)    # 1110011 (A0+A1=VDD)
#/*=========================================================================*/

#/*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
TCA9545_REG_CONFIG            =          (0x00)
#    /*---------------------------------------------------------------------*/

TCA9545_CONFIG_BUS0  =                (0x01)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS1  =                (0x02)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS2  =                (0x04)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS3  =                (0x08)  # 1 = enable, 0 = disable 

#/*=========================================================================*/

# I2C Mux TCA9545 Detection
try:
    tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS0)


    # turn I2CBus 1 on
    tca9545.write_control_register(TCA9545_CONFIG_BUS3)
    TCA9545_I2CMux_Present = True
except:
    print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"
    print "TCA9545 I2C Mux Not Present" 
    print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"



# Default constructor will pick a default I2C bus.
#
# For the Raspberry Pi this means you should hook up to the only exposed I2C bus
# from the main GPIO header and the library will figure out the bus number based
# on the Pi's revision.
#
# For the Beaglebone Black the library will assume bus 1 by default, which is
# exposed with SCL = P9_19 and SDA = P9_20.
indoor =0
sensor = SDL_Pi_SI1145.SDL_Pi_SI1145(indoor=indoor)

time.sleep(1.0)



print 'Press Cntrl + Z to cancel'

while True:
        vis = sensor.readVisible()
        IR = sensor.readIR()
        UV = sensor.readUV()
        IR_Lux = SI1145Lux.SI1145_IR_to_Lux(IR)
        vis_Lux = SI1145Lux.SI1145_VIS_to_Lux(vis)
        uvIndex = UV / 100.0
	print '--------------------'
	print '--------------------'
        print 'indoor=', indoor
	print '--------------------'
	print 'Vis:             ' + str(vis)
        print 'IR:              ' + str(IR)
	print 'UV:		' + str(UV)
	print '--------------------'
        print 'Vis Lux:             ' + str(vis_Lux)
        print 'IR Lux:              ' + str(IR_Lux)
        print 'UV Index:        ' + str(uvIndex)
	print '--------------------'
	print '--------------------'
        time.sleep(5)

