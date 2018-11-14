#!/usr/bin/env python

import AM2315

while (1):
	thsen = AM2315.AM2315()
	print "T   ", thsen.read_temperature()
	print "H   ", thsen.read_humidity()
	print "H,T ", thsen.read_humidity_temperature()

