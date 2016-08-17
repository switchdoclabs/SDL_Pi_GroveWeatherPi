#!/usr/bin/env python
#
# Test SDL_Pi_FRAM
# SwitchDoc Labs
# For WeatherPiArduino
# February 16, 2015 
#
#

# imports

import sys
import time
import random 
import SDL_Pi_FRAM

# Main Program

print ""
print "Test SDL_Pi_FRAM Version 1.0 - SwitchDoc Labs"
print ""
print ""
print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")
print ""

fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr = 0x50)

while True:
	print "----------------- "
	print "----------------- "
	print " Reading and Writing to FRAM "
	print "----------------- "
	print "writing first 10 addresses with random data"
	for x in range(0,10):
		value = random.randint(0,255)
		print "address = %i writing value=%i" % (x, value) 	
		fram.write8(x, value)
	print "----------------- "

	print "reading first 10 addresses"
	for x in range(0,10):
		print "address = %i value = %i" %(x, fram.read8(x)) 
	print "----------------- "
	print "----------------- "
	
	time.sleep(10.0)


