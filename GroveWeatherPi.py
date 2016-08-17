#!/usr/bin/env python
#
# GroveWeatherPi Solar Powered Weather Station
# Version 1.4 April 11, 2015 
#
# SwitchDoc Labs
# www.switchdoc.com
#
#

# imports

import sys
import time
from datetime import datetime
import random 
import re
import math
import os

import sendemail
import pclogging
import MySQLdb as mdb


sys.path.append('./SDL_Pi_SSD1306')
sys.path.append('./Adafruit_Python_SSD1306')
sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./SDL_Pi_FRAM')
sys.path.append('./RaspberryPi-AS3935/RPi_AS3935')
sys.path.append('./SDL_Pi_INA3221')
sys.path.append('./SDL_Pi_TCA9545')
sys.path.append('./graphs')

import subprocess
import RPi.GPIO as GPIO
import doAllGraphs




# Check for user imports
try:
	import conflocal as config
except ImportError:
	import config



################
# Device Present State Variables
###############

#indicate interrupt has happened from as3936

as3935_Interrupt_Happened = False;
# set to true if you are building the Weather Board project with Lightning Sensor
config.Lightning_Mode = True

# set to true if you are building the solar powered version
config.SolarPower_Mode = True;

config.TCA9545_I2CMux_Present = False
config.SunAirPlus_Present = False
config.AS3935_Present = False
config.DS3231_Present = False
config.BMP280_Present = False
config.FRAM_Present = False
config.HTU21DF_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False
config.OLED_Present = False




import SDL_Pi_INA3221
import SDL_DS3231
import Adafruit_BMP.BMP280 as BMP280
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack

import SDL_Pi_FRAM
from RPi_AS3935 import RPi_AS3935

import SDL_Pi_TCA9545

import Adafruit_SSD1306

import Scroll_SSD1306


def returnStatusLine(device, state):

        returnString = device
        if (state == True):
                returnString = returnString + ":   \t\tPresent"
        else:
                returnString = returnString + ":   \t\tNot Present"
        return returnString




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
	tca9545.write_control_register(TCA9545_CONFIG_BUS2)
	config.TCA9545_I2CMux_Present = True
except:
	print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"
	print "TCA9545 I2C Mux Not Present" 
	print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"
	config.TCA9545_I2CMux_Present = False

################
# SunAirPlus Sensors


# the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
LIPO_BATTERY_CHANNEL = 1
SOLAR_CELL_CHANNEL   = 2
OUTPUT_CHANNEL       = 3

try:
         # switch to BUS2 -  SunAirPlus is on Bus2
         tca9545.write_control_register(TCA9545_CONFIG_BUS2)
         sunAirPlus = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)

         busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
         config.SunAirPlus_Present = True
except:
         config.SunAirPlus_Present = False



SUNAIRLED = 25



################
# turn I2CBus 0 on
if (config.TCA9545_I2CMux_Present):
	 tca9545.write_control_register(TCA9545_CONFIG_BUS0)

# HTU21DF Detection
try:
        HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
        config.HTU21DF_Present = True
except:
        config.HTU21DF_Present = False


###############

#WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#

anemometerPin = 26
rainPin = 21

# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1    # internally, the library checks for ADS1115 or ADS1015 if found

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

# turn I2CBus 0 on
if (config.TCA9545_I2CMux_Present):
	tca9545.write_control_register(TCA9545_CONFIG_BUS0)
weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(anemometerPin, rainPin, 0,0, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
#weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)

################
# DS3231/AT24C32 Setup
# turn I2CBus 0 on
if (config.TCA9545_I2CMux_Present):
         tca9545.write_control_register(TCA9545_CONFIG_BUS0)

filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
starttime = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)


try:

        #comment out the next line after the clock has been initialized
        #ds3231.write_now()
        ds3231.read_datetime()
        #print "DS3231=\t\t%s" % ds3231.read_datetime()
        config.DS3231_Present = True
        #print "----------------- "
        #print "----------------- "
        #print " AT24C32 EEPROM"
        #print "----------------- "
        #print "writing first 4 addresses with random data"
        for x in range(0,4):
                value = random.randint(0,255)
        	#print "address = %i writing value=%i" % (x, value)
                ds3231.write_AT24C32_byte(x, value)
        #print "----------------- "

        #print "reading first 4 addresses"
        #for x in range(0,4):
        #        print "address = %i value = %i" %(x, ds3231.read_AT24C32_byte(x))
        #print "----------------- "

except IOError as e:
        #print "I/O error({0}): {1}".format(e.errno, e.strerror)
        config.DS3231_Present = False



################

# BMP280 Setup 

try:
        bmp280 = BMP280.BMP280()
        config.BMP280_Present = True

except IOError as e:

        #    print "I/O error({0}): {1}".format(e.errno, e.strerror)
        config.BMP280_Present = False

################

# OLED SSD_1306 Detection

try:
        RST =27
        display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
        # Initialize library.
        display.begin()
        display.clear()
        display.display()
        config.OLED_Present = True
except:
        config.OLED_Present = False

################

# ad3935 Set up Lightning Detector
if (config.Lightning_Mode == True):
        # switch to BUS1 - lightning detector is on Bus1
        if (config.TCA9545_I2CMux_Present):
         	tca9545.write_control_register(TCA9545_CONFIG_BUS1)

        as3935 = RPi_AS3935(address=0x03, bus=1)

        try:

                as3935.set_indoors(False)
                config.AS3935_Present = True
                #print "as3935 present"

        except IOError as e:

                #    print "I/O error({0}): {1}".format(e.errno, e.strerror)
                config.AS3935_Present = False
                # back to BUS0
                if (config.TCA9545_I2CMux_Present):
        		 tca9545.write_control_register(TCA9545_CONFIG_BUS0)


        if (config.AS3935_Present == True):
                #i2ccommand = "sudo i2cdetect -y 1"
                #output = subprocess.check_output (i2ccommand,shell=True, stderr=subprocess.STDOUT )
                #print output
                as3935.set_noise_floor(0)
                as3935.calibrate(tun_cap=0x0F)

        as3935LastInterrupt = 0
        as3935LightningCount = 0
        as3935LastDistance = 0
        as3935LastStatus = ""
	as3935Interrupt = False
        # back to BUS0
        if (config.TCA9545_I2CMux_Present):
        	 tca9545.write_control_register(TCA9545_CONFIG_BUS0)
	time.sleep(0.003)



def process_as3935_interrupt():

    global as3935Interrupt
    global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus

    as3935Interrupt = False

    print "processing Interrupt from as3935"
    # turn I2CBus 2 on
    if (config.TCA9545_I2CMux_Present):
         tca9545.write_control_register(TCA9545_CONFIG_BUS1)
    time.sleep(0.003)
    reason = as3935.get_interrupt()

    as3935LastInterrupt = reason
    
    if reason == 0x00:
	as3935LastStatus = "Spurious Interrupt"
    elif reason == 0x01:
	as3935LastStatus = "Noise Floor too low. Adjusting"
        as3935.raise_noise_floor()
    elif reason == 0x04:
	as3935LastStatus = "Disturber detected - masking"
        as3935.set_mask_disturber(True)
    elif reason == 0x08:
        now = datetime.now().strftime('%H:%M:%S - %Y/%m/%d')
        distance = as3935.get_distance()
	as3935LastDistance = distance
	as3935LastStatus = "Lightning Detected "  + str(distance) + "km away. (%s)" % now
	pclogging.log(pclogging.INFO, __name__, "Lightning Detected "  + str(distance) + "km away. (%s)" % now)
	sendemail.sendEmail("test", "GroveWeatherPi Lightning Detected\n", as3935LastStatus, config.textnotifyAddress,  config.textfromAddress, "");
    
    print "Last Interrupt = 0x%x:  %s" % (as3935LastInterrupt, as3935LastStatus)
    if (config.TCA9545_I2CMux_Present):
         tca9545.write_control_register(TCA9545_CONFIG_BUS0)

    time.sleep(0.003)



def handle_as3935_interrupt(channel):
    global as3935Interrupt

    print "as3935 Interrupt"

    as3935Interrupt = True


# define Interrupt Pin for AS3935
as3935pin = 13 

GPIO.setup(as3935pin, GPIO.IN)
#GPIO.setup(as3935pin, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)


##############
# Setup AM2315
# turn I2CBus 0 on
if (config.TCA9545_I2CMux_Present):
         tca9545.write_control_register(TCA9545_CONFIG_BUS0)


###############

# Detect AM2315
try:
        from tentacle_pi.AM2315 import AM2315
        try:
		am2315 = AM2315(0x5c,"/dev/i2c-1")
    		outsideTemperature, outsideHumidity, crc_check = am2315.sense() 
		#print "outsideTemperature: %0.1f C" % outsideTemperature
    		#print "outsideHumidity: %0.1f %%" % outsideHumidity
    		#print "crc: %s" % crc_check
                config.AM2315_Present = True
		if (crc_check == -1):
                	config.AM2315_Present = False

        except:
                config.AM2315_Present = False
except:
        config.AM2315_Present = False
        print "------> See Readme to install tentacle_pi"

config.AM2315_Present = True



###############

# Set up FRAM 
# Set up FRAM

# turn I2CBus 0 on
if (config.TCA9545_I2CMux_Present):
         tca9545.write_control_register(TCA9545_CONFIG_BUS0)
fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr = 0x50)
# FRAM Detection
try:
        fram.read8(0)
        config.FRAM_Present = True
except:
        config.FRAM_Present = False


#fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr = 0x50)


# Main Loop - sleeps 10 seconds
# command from RasPiConnect Execution Code

def completeCommand():

        f = open("/home/pi/SDL_Pi_GroveWeatherPi/state/WeatherCommand.txt", "w")
        f.write("DONE")
        f.close()

def completeCommandWithValue(value):

        f = open("/home/pi/SDL_Pi_GroveWeatherPi/state/WeatherCommand.txt", "w")
        f.write(value)
        f.close()

def processCommand():

        f = open("//home/pi/SDL_Pi_GroveWeatherPi/state/WeatherCommand.txt", "r")
        command = f.read()
        f.close()

	if (command == "") or (command == "DONE"):
		# Nothing to do
		return False

	# Check for our commands
	#pclogging.log(pclogging.INFO, __name__, "Command %s Recieved" % command)

	print "Processing Command: ", command
	if (command == "SAMPLEWEATHER"):
		sampleWeather()
		completeCommand()
	    	writeWeatherStats()
		return True

	if (command == "SAMPLEBOTH"):
		sampleWeather()
		completeCommand()
	    	writeWeatherStats()
		sampleSunAirPlus()
	    	writeSunAirPlusStats()
		return True

	if (command == "SAMPLEBOTHGRAPHS"):
		sampleWeather()
		completeCommand()
	    	writeWeatherStats()
		sampleSunAirPlus()
	    	writeSunAirPlusStats()
		doAllGraphs.doAllGraphs()
		return True
			
			
			
	completeCommand()


	return False


# Main Program


def returnPercentLeftInBattery(currentVoltage, maxVolt):

	scaledVolts = currentVoltage / maxVolt
	
	if (scaledVolts > 1.0):
		scaledVolts = 1.0
	
	
	if (scaledVolts > .9686):
		returnPercent = 10*(1-(1.0-scaledVolts)/(1.0-.9686))+90
		return returnPercent

	if (scaledVolts > 0.9374):
		returnPercent = 10*(1-(0.9686-scaledVolts)/(0.9686-0.9374))+80
		return returnPercent


	if (scaledVolts > 0.9063):
		returnPercent = 30*(1-(0.9374-scaledVolts)/(0.9374-0.9063))+50
		return returnPercent

	if (scaledVolts > 0.8749):
		returnPercent = 30*(1-(0.8749-scaledVolts)/(0.9063-0.8749))+20
		return returnPercent

	
	if (scaledVolts > 0.8437):
		returnPercent = 17*(1-(0.8437-scaledVolts)/(0.8749-0.8437))+3
		return returnPercent


   	if (scaledVolts > 0.8126):
		returnPercent = 1*(1-(0.8126-scaledVolts)/(0.8437-0.8126))+2
		return returnPercent



	if (scaledVolts > 0.7812):
		returnPercent = 1*(1-(0.7812-scaledVolts)/(0.7812-0.8126))+1
		return returnPercent

	return 0	

# write SunAirPlus stats out to file
def writeSunAirPlusStats():

        f = open("/home/pi/GroveWeatherPiSolarPoweredWeather/state/SunAirPlusStats.txt", "w")
	f.write(str(batteryVoltage) + '\n')
	f.write(str(batteryCurrent ) + '\n')
	f.write(str(solarVoltage) + '\n')
	f.write(str(solarCurrent ) + '\n')
	f.write(str(loadVoltage ) + '\n')
	f.write(str(loadCurrent) + '\n')
	f.write(str(batteryPower ) + '\n')
	f.write(str(solarPower) + '\n')
	f.write(str(loadPower) + '\n')
	f.write(str(batteryCharge) + '\n')
        f.close()

# write weather stats out to file
def writeWeatherStats():

        f = open("/home/pi/GroveWeatherPiSolarPoweredWeather/state/WeatherStats.txt", "w")
	f.write(str(totalRain) + '\n') 
	f.write(str(as3935LightningCount) + '\n')
	f.write(str(as3935LastInterrupt) + '\n')
	f.write(str(as3935LastDistance) + '\n')
	f.write(str(as3935LastStatus) + '\n')
 	f.write(str(currentWindSpeed) + '\n')
	f.write(str(currentWindGust) + '\n')
	f.write(str(totalRain)  + '\n')
  	f.write(str(bmp180Temperature)  + '\n')
	f.write(str(bmp180Pressure) + '\n')
	f.write(str(bmp180Altitude) + '\n')
	f.write(str(bmp180SeaLevel)  + '\n')
    	f.write(str(outsideTemperature) + '\n')
	f.write(str(outsideHumidity) + '\n')
	f.write(str(currentWindDirection) + '\n')
	f.write(str(currentWindDirectionVoltage) + '\n')
	f.write(str(HTUtemperature) + '\n')
	f.write(str(HTUhumidity) + '\n')
        f.close()



# sample and display
totalRain = 0
def sampleWeather():

	global as3935LightningCount
    	global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
 	global currentWindSpeed, currentWindGust, totalRain 
  	global 	bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel 
    	global outsideTemperature, outsideHumidity, crc_check 
	global currentWindDirection, currentWindDirectionVoltage

	global HTUtemperature, HTUhumidity

        # blink GPIO LED when it's run
        GPIO.setup(SUNAIRLED, GPIO.OUT)
        GPIO.output(SUNAIRLED, True)
        time.sleep(0.2)
        GPIO.output(SUNAIRLED, False)

	print "----------------- "
	print " Weather Sampling" 
	print "----------------- "
	#
	# turn I2CBus 0 on
 	if (config.TCA9545_I2CMux_Present):
         	tca9545.write_control_register(TCA9545_CONFIG_BUS0)

 	currentWindSpeed = weatherStation.current_wind_speed()/1.6
  	currentWindGust = weatherStation.get_wind_gust()/1.6
  	totalRain = totalRain + weatherStation.get_current_rain_total()/25.4
	currentWindDirection = weatherStation.current_wind_direction()
	currentWindDirectionVoltage = weatherStation.current_wind_direction_voltage()
  
	if (config.BMP280_Present):	
		bmp180Temperature = bmp280.read_temperature()
		bmp180Pressure = bmp280.read_pressure()/1000
		bmp180Altitude = bmp280.read_altitude()
		bmp180SeaLevel = bmp280.read_sealevel_pressure()/1000



	if (config.HTU21DF_Present):
		# We use a C library for this device as it just doesn't play well with Python and smbus/I2C libraries
		HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
		splitstring = HTU21DFOut.split()

		HTUtemperature = float(splitstring[0])	
		HTUhumidity = float(splitstring[1])	


	if (as3935LastInterrupt == 0x00):
		as3935InterruptStatus = "----No Lightning detected---"
		
	if (as3935LastInterrupt == 0x01):
		as3935InterruptStatus = "Noise Floor: %s" % as3935LastStatus
		as3935LastInterrupt = 0x00

	if (as3935LastInterrupt == 0x04):
		as3935InterruptStatus = "Disturber: %s" % as3935LastStatus
		as3935LastInterrupt = 0x00

	if (as3935LastInterrupt == 0x08):
		as3935InterruptStatus = "Lightning: %s" % as3935LastStatus
		as3935LightningCount += 1
		as3935LastInterrupt = 0x00


	if (config.AS3935_Present):
		as3935InterruptStatus = "No AS3935 Lightning Detector Present"
		as3935LastInterrupt = 0x00
		
	if (config.AM2315_Present):
		# get AM2315 Outside Humidity and Outside Temperature
		# turn I2CBus 0 on
 		if (config.TCA9545_I2CMux_Present):
        		 tca9545.write_control_register(TCA9545_CONFIG_BUS0)


    		outsideTemperature, outsideHumidity, crc_check = am2315.sense()


def sampleSunAirPlus():

	global batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent
	global batteryPower, solarPower, loadPower, batteryCharge


	if (config.SunAirPlus_Present):
		
		# turn I2CBus 2 on
 		if (config.TCA9545_I2CMux_Present):
        		 tca9545.write_control_register(TCA9545_CONFIG_BUS2)


		print "----------------- "
		print " SunAirPlus Sampling" 
		print "----------------- "
		#
        	# blink GPIO LED when it's run
        	GPIO.setup(SUNAIRLED, GPIO.OUT)
        	GPIO.output(SUNAIRLED, True)
        	time.sleep(0.2)
        	GPIO.output(SUNAIRLED, False)
	

	
        	busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
        	shuntvoltage1 = sunAirPlus.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL)
        	# minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
        	batteryCurrent = sunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL)
        	batteryVoltage = busvoltage1 + (shuntvoltage1 / 1000)
		batteryPower = batteryVoltage * (batteryCurrent/1000)


        	busvoltage2 = sunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL)
        	shuntvoltage2 = sunAirPlus.getShuntVoltage_mV(SOLAR_CELL_CHANNEL)
        	solarCurrent = -sunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL)
        	solarVoltage = busvoltage2 + (shuntvoltage2 / 1000)
		solarPower = solarVoltage * (solarCurrent/1000)

        	busvoltage3 = sunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL)
        	shuntvoltage3 = sunAirPlus.getShuntVoltage_mV(OUTPUT_CHANNEL)
        	loadCurrent = sunAirPlus.getCurrent_mA(OUTPUT_CHANNEL)
        	loadVoltage = busvoltage3 
		loadPower = loadVoltage * (loadCurrent/1000)

		batteryCharge = returnPercentLeftInBattery(batteryVoltage, 4.19)	

	else:
	
		print "----------------- "
		print " SunAirPlus Not Present" 
		print "----------------- "

def sampleAndDisplay():

	global totalRain, as3935LightningCount
    	global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
	
	# turn I2CBus 0 on
 	if (config.TCA9545_I2CMux_Present):
         	 tca9545.write_control_register(TCA9545_CONFIG_BUS0)
	
	print "----------------- "
	print " WeatherRack Weather Sensors Sampling" 
	print "----------------- "
	#

 	currentWindSpeed = weatherStation.current_wind_speed()/1.6
  	currentWindGust = weatherStation.get_wind_gust()/1.6
  	totalRain = totalRain + weatherStation.get_current_rain_total()/25.4
  	print("Rain Total=\t%0.2f in")%(totalRain)
  	print("Wind Speed=\t%0.2f MPH")%(currentWindSpeed)
    	print("MPH wind_gust=\t%0.2f MPH")%(currentWindGust)
  	
        if (config.ADS1015_Present or config.ADS1115_Present):
		print "Wind Direction=\t\t\t %0.2f Degrees" % weatherStation.current_wind_direction()
		print "Wind Direction Voltage=\t\t %0.3f V" % weatherStation.current_wind_direction_voltage()
	else:
		print "No Wind Direction Available - No ADS1015 or ADS1115 Present"

        if (config.OLED_Present):
                Scroll_SSD1306.addLineOLED(display,  ("Wind Speed=\t%0.2f MPH")%(currentWindSpeed))
                Scroll_SSD1306.addLineOLED(display,  ("Rain Total=\t%0.2f in")%(totalRain))
                if (config.ADS1015_Present or config.ADS1115_Present):
                        Scroll_SSD1306.addLineOLED(display,  "Wind Dir=%0.2f Degrees" % weatherStation.current_wind_direction())
	
	print "----------------- "
        print "----------------- "
        if (config.DS3231_Present == True):
                print " DS3231 Real Time Clock"
        else:
                print " DS3231 Real Time Clock Not Present"

        print "----------------- "
        #

        if (config.DS3231_Present == True):
                currenttime = datetime.utcnow()

                deltatime = currenttime - starttime

                print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")

                if (config.OLED_Present):
                        Scroll_SSD1306.addLineOLED(display,"%s" % ds3231.read_datetime())

                print "DS3231=\t\t%s" % ds3231.read_datetime()

                print "DS3231 Temperature= \t%0.2f C" % ds3231.getTemp()
                print "----------------- "



	print "----------------- "
        print "----------------- "
        if (config.BMP280_Present == True):
                print " BMP280 Barometer"
        else:
                print " BMP280 Barometer Not Present"
        print "----------------- "

	if (config.BMP280_Present):
		print 'Temperature = \t{0:0.2f} C'.format(bmp280.read_temperature())
		print 'Pressure = \t{0:0.2f} KPa'.format(bmp280.read_pressure()/1000)
		print 'Altitude = \t{0:0.2f} m'.format(bmp280.read_altitude())
		print 'Sealevel Pressure = \t{0:0.2f} KPa'.format(bmp280.read_sealevel_pressure()/1000)
		print "----------------- "


        print "----------------- "
        if (config.HTU21DF_Present == True):
                print " HTU21DF Temp/Hum"
        else:
                print " HTU21DF Temp/Hum Not Present"
        print "----------------- "

        # We use a C library for this device as it just doesn't play well with Python and smbus/I2C libraries
        if (config.HTU21DF_Present):
                HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
                splitstring = HTU21DFOut.split()

                HTUtemperature = float(splitstring[0])
                HTUhumidity = float(splitstring[1])
                print "Temperature = \t%0.2f C" % HTUtemperature
                print "Humidity = \t%0.2f %%" % HTUhumidity
                if (config.OLED_Present):
                        Scroll_SSD1306.addLineOLED(display,  "InTemp = \t%0.2f C" % HTUtemperature)
        print "----------------- "

	print "----------------- "
	if (config.AS3935_Present):
		print " AS3935 Lightning Detector "
	else:
		print " AS3935 Lightning Detector Not Present "

	print "----------------- "

	if (config.AS3935_Present):
		print "Last result from AS3935:"

		if (as3935LastInterrupt == 0x00):
			print "----No Lightning detected---"
		
		if (as3935LastInterrupt == 0x01):
			print "Noise Floor: %s" % as3935LastStatus
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x04):
			print "Disturber: %s" % as3935LastStatus
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x08):
			print "Lightning: %s" % as3935LastStatus
	        	if (config.OLED_Present):
                       	         Scroll_SSD1306.addLineOLED(display, '')
                       	         Scroll_SSD1306.addLineOLED(display, '---LIGHTNING---')
                                 Scroll_SSD1306.addLineOLED(display, '')
			as3935LightningCount += 1
			as3935LastInterrupt = 0x00

		print "Lightning Count = ", as3935LightningCount

	print "----------------- "

	# turn I2CBus 0 on
 	if (config.TCA9545_I2CMux_Present):
         	tca9545.write_control_register(TCA9545_CONFIG_BUS0)


        print "----------------- "
        if (config.AM2315_Present == True):
                print " AM2315 Temperature/Humidity Sensor"
        else:
                print " AM2315 Temperature/Humidity Sensor Not Present"
        print "----------------- "

        if (config.AM2315_Present):
    		outsideTemperature, outsideHumidity, crc_check = am2315.sense()
    		print "outsideTemperature: %0.1f C" % outsideTemperature
    		print "outsideHumidity: %0.1f %%" % outsideHumidity
    		print "crc: %s" % crc_check
        print "----------------- "

	if (config.SunAirPlus_Present):
        
		# turn I2CBus 2 on
 		tca9545.write_control_register(TCA9545_CONFIG_BUS2)

		print "----------------- "
		print "----------------- "
		print "----------------- "
		print "SunAirPlus Currents / Voltage "
		print "----------------- "
        	shuntvoltage1 = 0
        	busvoltage1   = 0
        	current_mA1   = 0
        	loadvoltage1  = 0


        	busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
        	shuntvoltage1 = sunAirPlus.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL)
        	# minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
        	current_mA1 = sunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL)

        	loadvoltage1 = busvoltage1  + (shuntvoltage1 / 1000)   
		batteryPower = loadvoltage1 * (current_mA1/1000)

        	print "LIPO_Battery Bus Voltage: %3.2f V " % busvoltage1
        	print "LIPO_Battery Shunt Voltage: %3.2f mV " % shuntvoltage1
        	print "LIPO_Battery Load Voltage:  %3.2f V" % loadvoltage1
        	print "LIPO_Battery Current 1:  %3.2f mA" % current_mA1
        	print "Battery Power 1:  %3.2f W" % batteryPower
        	print

        	shuntvoltage2 = 0
        	busvoltage2 = 0
        	current_mA2 = 0
        	loadvoltage2 = 0

        	busvoltage2 = sunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL)
        	shuntvoltage2 = sunAirPlus.getShuntVoltage_mV(SOLAR_CELL_CHANNEL)
        	current_mA2 = -sunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL)
        	loadvoltage2 = busvoltage2  + (shuntvoltage2 / 1000)
		solarPower = loadvoltage2 * (current_mA2/1000)

        	print "Solar Cell Bus Voltage 2:  %3.2f V " % busvoltage2
        	print "Solar Cell Shunt Voltage 2: %3.2f mV " % shuntvoltage2
        	print "Solar Cell Load Voltage 2:  %3.2f V" % loadvoltage2
        	print "Solar Cell Current 2:  %3.2f mA" % current_mA2
        	print "Solar Cell Power 2:  %3.2f W" % solarPower
        	print

        	shuntvoltage3 = 0
        	busvoltage3 = 0
        	current_mA3 = 0
        	loadvoltage3 = 0

        	busvoltage3 = sunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL)
        	shuntvoltage3 = sunAirPlus.getShuntVoltage_mV(OUTPUT_CHANNEL)
        	current_mA3 = sunAirPlus.getCurrent_mA(OUTPUT_CHANNEL)
        	loadvoltage3 = busvoltage3 
		loadPower = loadvoltage3 * (current_mA3/1000)

        	print "Output Bus Voltage 3:  %3.2f V " % busvoltage3
        	print "Output Shunt Voltage 3: %3.2f mV " % shuntvoltage3
        	print "Output Load Voltage 3:  %3.2f V" % loadvoltage3
        	print "Output Current 3:  %3.2f mA" % current_mA3
        	print "Output Power 3:  %3.2f W" % loadPower
        	print

	        print "------------------------------"




def writeWeatherRecord():
	global as3935LightningCount
    	global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
 	global currentWindSpeed, currentWindGust, totalRain 
  	global 	bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel 
    	global outsideTemperature, outsideHumidity, crc_check 
	global currentWindDirection, currentWindDirectionVoltage

	global HTUtemperature, HTUhumidity


	# now we have the data, stuff it in the database

	try:
		print("trying database")
    		con = mdb.connect('localhost', 'root', DATABASEPASSWORD, 'GroveWeatherPi');

    		cur = con.cursor()
		print "before query"

		query = 'INSERT INTO WeatherData(TimeStamp,as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, insideTemperature, insideHumidity) VALUES(UTC_TIMESTAMP(), %.3f, %.3f, %.3f, "%s", %.3f, %.3f, %.3f, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, HTUtemperature, HTUhumidity)
		print("query=%s" % query)

		cur.execute(query)
	
		con.commit()
		
	except mdb.Error, e:
  
    		print "Error %d: %s" % (e.args[0],e.args[1])
    		con.rollback()
    		#sys.exit(1)
    
	finally:    
       		cur.close() 
        	con.close()

		del cur
		del con





def writePowerRecord():

	# now we have the data, stuff it in the database

	try:
		print("trying database")
    		con = mdb.connect('localhost', 'root', DATABASEPASSWORD, 'GroveWeatherPi');

    		cur = con.cursor()
		print "before query"

		query = 'INSERT INTO PowerSystem(TimeStamp, batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent, batteryPower, solarPower, loadPower, batteryCharge) VALUES (UTC_TIMESTAMP (), %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent, batteryPower, solarPower, loadPower, batteryCharge) 
		
		print("query=%s" % query)

		cur.execute(query)
	
		con.commit()
		
	except mdb.Error, e:
  
    		print "Error %d: %s" % (e.args[0],e.args[1])
    		con.rollback()
    		#sys.exit(1)
    
	finally:    
       		cur.close() 
        	con.close()

		del cur
		del con


WATCHDOGTRIGGER = 17
def patTheDog():


	# pat the dog
	print "------Patting The Dog------- "
        GPIO.setup(WATCHDOGTRIGGER, GPIO.OUT)
        GPIO.output(WATCHDOGTRIGGER, False)
        time.sleep(0.2)
        GPIO.output(WATCHDOGTRIGGER, True)
        GPIO.setup(WATCHDOGTRIGGER, GPIO.IN)


	
def shutdownPi(why):

   pclogging.log(pclogging.INFO, __name__, "Pi Shutting Down: %s" % why)
   sendemail.sendEmail("test", "GroveWeatherPi Shutting down:"+ why, "The GroveWeatherPi Raspberry Pi shutting down.", config.notifyAddress,  config.fromAddress, "");
   sys.stdout.flush()
   time.sleep(10.0)

   os.system("sudo shutdown -h now")

def rebootPi(why):

   pclogging.log(pclogging.INFO, __name__, "Pi Rebooting: %s" % why)
   os.system("sudo shutdown -r now")

def blinkSunAirLED2X(howmany):

   # blink GPIO LED when it's run
   GPIO.setup(SUNAIRLED, GPIO.OUT)

   i = 0
   while (i< howmany):
   	GPIO.output(SUNAIRLED, True)
   	time.sleep(0.2)
   	GPIO.output(SUNAIRLED, False)
   	time.sleep(0.2)
	i = i +1



import urllib2 


def checkInternetConnection():
    try:
        urllib2.urlopen("http://www.google.com").close()
    except urllib2.URLError:
        print "Internet Not Connected"
        time.sleep(1)
	return false
    else:
        print "Internet Connected"
	return true


WLAN_check_flg = 0

def WLAN_check():
        '''
        This function checks if the WLAN is still up by pinging the router.
        If there is no return, we'll reset the WLAN connection.
        If the resetting of the WLAN does not work, we need to reset the Pi.
        source http://www.raspberrypi.org/forums/viewtopic.php?t=54001&p=413095
        '''
	global WLAN_check_flg
        ping_ret = subprocess.call(['ping -c 2 -w 1 -q 192.168.1.1 |grep "1 received" > /dev/null 2> /dev/null'], shell=True)
	if ping_ret:
            # we lost the WLAN connection.
            # did we try a recovery already?
            if (WLAN_check_flg>2):
                # we have a serious problem and need to reboot the Pi to recover the WLAN connection
		print "logger WLAN Down, Pi is forcing a reboot"
   		pclogging.log(pclogging.ERROR, __name__, "WLAN Down, Pi is forcing a reboot")
                WLAN_check_flg = 0 
		rebootPi("WLAN Down")
                #subprocess.call(['sudo shutdown -r now'], shell=True)
            else:
                # try to recover the connection by resetting the LAN
                #subprocess.call(['logger "WLAN is down, Pi is resetting WLAN connection"'], shell=True)
		print "WLAN Down, Pi is trying resetting WLAN connection"
   		pclogging.log(pclogging.WARNING, __name__, "WLAN Down, Pi is resetting WLAN connection" )
                WLAN_check_flg = WLAN_check_flg + 1 # try to recover
                subprocess.call(['sudo /sbin/ifdown wlan0 && sleep 10 && sudo /sbin/ifup --force wlan0'], shell=True)
        else:
            WLAN_check_flg = 0
	    print "WLAN is OK"




print ""
print "GroveWeatherPi Solar Powered Weather Station Version 2.0 - SwitchDoc Labs"
print ""
print ""
print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")
print ""



print "----------------------"
print returnStatusLine("I2C Mux - TCA9545",config.TCA9545_I2CMux_Present)
print returnStatusLine("DS3231",config.DS3231_Present)
print returnStatusLine("BMP280",config.BMP280_Present)
print returnStatusLine("FRAM",config.FRAM_Present)
print returnStatusLine("HTU21DF",config.HTU21DF_Present)
print returnStatusLine("AM2315",config.AM2315_Present)
print returnStatusLine("ADS1015",config.ADS1015_Present)
print returnStatusLine("ADS1115",config.ADS1115_Present)
print returnStatusLine("AS3935",config.AS3935_Present)
print returnStatusLine("OLED",config.OLED_Present)
print returnStatusLine("SunAirPlus",config.SunAirPlus_Present)
print "----------------------"

DATABASEPASSWORD = "password"
pclogging.log(pclogging.INFO, __name__, "GroveWeatherPi Startup Version 2.0")

sendemail.sendEmail("test", "GroveWeatherPi Startup \n", "The GroveWeatherPi Raspberry Pi has #rebooted.", config.notifyAddress,  config.fromAddress, "");




secondCount = 1
while True:
	
	# process Interrupts from Lightning

	if (as3935Interrupt == True):

		try:
			process_as3935_interrupt()

			
		except:
			print "exception - as3935 I2C did not work"


 	if (config.TCA9545_I2CMux_Present):
         	tca9545.write_control_register(TCA9545_CONFIG_BUS0)
	# process commands from RasPiConnect
	print "---------------------------------------- "

	processCommand()	

	if ((secondCount % 10) == 0):
		# print every 10 seconds
		sampleAndDisplay()		
		patTheDog()      # reset the WatchDog Timer
		blinkSunAirLED2X(2)




	# every 5 minutes, push data to mysql and check for shutdown


	if ((secondCount % (5*60)) == 0):
		# print every 300 seconds
                sampleWeather()
                sampleSunAirPlus()
		writeWeatherRecord()
		writePowerRecord()

		if (batteryVoltage < 3.5):
			print "--->>>>Time to Shutdown<<<<---"
			shutdownPi("low voltage shutdown")


	# every 15 minutes, build new graphs

	if ((secondCount % (15*60)) == 0):
		# print every 900 seconds
                sampleWeather()
                sampleSunAirPlus()
		doAllGraphs.doAllGraphs()

	# every 30 minutes, check wifi connections 

	if ((secondCount % (30*60)) == 0):
		# print every 900 seconds
    		WLAN_check()

    	#WLAN_check()


	# every 48 hours, reboot
	if ((secondCount % (60*60*48)) == 0):
		# reboot every 48() hours seconds
		rebootPi("48 hour reboot")		


	secondCount = secondCount + 1
	# reset secondCount to prevent overflow forever

	if (secondCount == 1000001):
		secondCount = 1	
	
	time.sleep(1.0)

