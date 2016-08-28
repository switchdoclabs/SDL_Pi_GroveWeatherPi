GroveWeatherPi Libraries and Example for Raspberry Pi Solar Powered Weather Station

Supports SwitchDoc Labs WeatherRack WeatherBoard (WeatherPiArduino V2 and above)

Version 2.1 

Other installations required for AM2315:

sudo apt-get install python-pip
sudo apt-get install libi2c-dev
sudo pip install tentacle_pi

SwitchDocLabs Documentation for WeatherRack/WeatherPiArduino under products on:

-----------
setup your configuration variables in config.py
-----------

--------
Add SQL instructions
----------

Use phpmyadmin or sql command lines to add the included SQL file to your MySQL databases.

example:   mysql -u root -p < WeatherPiStructure.sql

user:  root
password: password

Obviously with these credentials, don't connect port 3306 to the Internet.   Change them if you aren't sure.

NOTE:

If you have a WXLink wireless transmitter installed, the software assumes you have connected your AM2315 outdoor temp/humidity sensor to the WXLink.  If you put another AM2315 on your local system, it will use those values instead of the WXLink values

----------

----------

http://www.switchdoc.com/

August 26, 2016 - Added Support for WXLink Wireless Weather Connector

August 17, 2016 -  Added support back in for RasPiConnect 

August 16, 2016 -  Support for Weather Board and improved device detection without exceptions

March 28, 2015 - added subdirectories

May 9, 2015 - Updated software for WatchDog Timer and Email

May 10, 2015 - Added mysql table SQL files for database building 

