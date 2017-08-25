GroveWeatherPi Libraries and Example for Raspberry Pi Solar Powered Weather Station

Supports SwitchDoc Labs WeatherRack WeatherBoard (WeatherPiArduino V2 and above)

Version 2.96 

http://www.switchdoc.com/

August 24, 2017 - Fixed AS3935 Missing  issue with Interrupt variable

July 20, 2017 - Fixed Rainfall 2X inaccuracy and 60 minute and midnight issues

July 17, 2017 - Updated Bounce Times to fix wind speed issues - This probably fixes the Wind Gust problem too.  Not conclusive.

June 19, 2017 - Fixed uninitialized variable errors

June 17, 2017 - Changed the README.md to require the installation of the software suporting SI1145 detection

May 5, 2017 - Commented out 48 Reboot add_job (uncomment to add back in), Fixed SI1145 bad installation detection 

May 4, 2017 - Fixed problem with inside temperature being zero when you have an HTU rather than a HDC1080 

March 9, 2017 - Added time based scheduler into the main loop instead of polling (APscheduler).  Added support for Grove HDC1000

October 26, 2016 - Support added for Grove Sunlight/IR/UV SI1145 sensor.   Database updated.  Run mysql as below

October 24, 2016 -  Improved WXLink Error Checking

October 3, 2016 - Added CRC Check to WXLink support, changed Barometric report on WU to Sea Level, added Altitude configuration in confif.py

September 9, 2016 - Added WeatherUnderground Support - see Blog article on www.switchdoc.com for instructions.   The summary of the instructions are:

1) sign up for a personal weather station on weatherunderground.com

2) Get your station name and key and put them in your config.py file, and then setting the WeatherUnderground_Present to True


August 30, 2016 - Improved WXLink support reliablity - now detects and adjusts for missing records from WXLink reads

August 26, 2016 - Added Support for WXLink Wireless Weather Connector

August 17, 2016 -  Added support back in for RasPiConnect 

August 16, 2016 -  Support for Weather Board and improved device detection without exceptions

March 28, 2015 - added subdirectories

May 9, 2015 - Updated software for WatchDog Timer and Email

May 10, 2015 - Added mysql table SQL files for database building 

-----------------
Install this for smbus:

sudo apt-get install python-smbus

Install this next:

git clone https://github.com/adafruit/Adafruit_Python_PureIO.git<BR>
cd Adafruit_Python_PureIO<BR>
sudo python setup.py install<BR>

Other installations required for AM2315:

sudo apt-get install python-pip

sudo apt-get install libi2c-dev

sudo pip install tentacle_pi

#Installing apscheduler

sudo pip install --upgrade setuptools pip <BR>

sudo pip install setuptools --upgrade  <BR>
sudo pip install apscheduler <BR>


----------------
Note some configurations of Raspberry Pi software requres the following:
----------------
<pre>
sudo apt-get update
sudo apt-get install build-essential python-pip python-dev python-smbus git
git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
cd Adafruit_Python_GPIO
sudo python setup.py install
</pre>
SwitchDocLabs Documentation for WeatherRack/WeatherPiArduino under products on: store.switchdoc.com

Read the GroveWeatherPi Instructable on instructables.com for more software installation instructions 

or

Read the tutorial on GroveWeatherPi on http://www.switchdoc.com/2016/08/tutorial-part-1-building-a-solar-powered-raspberry-pi-weather-station-groveweatherpi/
for more software installation instructions.

-----------
setup your configuration variables in config.py!
-----------

--------
Add SQL instructions
----------

Use phpmyadmin or sql command lines to add the included SQL file to your MySQL databases.<BR>
Note:  If the database has been updated, run the example below to update your database.   The current contents will not be lost.


example:   mysql -u root -p GroveWeatherPi< GroveWeatherPi.sql

user:  root

password: password

Obviously with these credentials, don't connect port 3306 to the Internet.   Change them if you aren't sure.

NOTE:

If you have a WXLink wireless transmitter installed, the software assumes you have connected your AM2315 outdoor temp/humidity sensor to the WXLink.  If you put another AM2315 on your local system, it will use those values instead of the WXLink values

----------

----------


