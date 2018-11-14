#!/usr/bin/env python

import AM2315

while (1):
	thsen = AM2315.AM2315()
	print "T   ", thsen.read_temperature()
	print "H   ", thsen.read_humidity()
	print "H,T ", thsen.read_humidity_temperature()
	print "H,T,C ", thsen.read_humidity_temperature_crc()
        h,t,c = thsen.read_humidity_temperature_crc()
        print "CRC=0x%02x" % c

