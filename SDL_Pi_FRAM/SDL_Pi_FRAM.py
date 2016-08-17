#!/usr/bin/env python

# SDL_Pi_FRAM.py Python Driver Code
# SwitchDoc Labs 02/16/2014
# V 1.2


from datetime import datetime

import time
import smbus



class SDL_Pi_FRAM():


    ###########################
    # SDL_Pi_FRAM Code
    ###########################
    def __init__(self, twi=1, addr=0x50):
        self._bus = smbus.SMBus(twi)
        self._addr = addr



    def write8(self, address, data):
        #print "addr =0x%x address = 0x%x data = 0x%x  " % (self._addr, address, data)
	self._bus.write_i2c_block_data(self._addr,address>>8,[address%256, data])



    def read8(self, address):

	self._bus.write_i2c_block_data(self._addr,address>>8,[address%256])
	returndata = self._bus.read_byte(self._addr) # this will read at the current address pointer, which we on the previous line
        #print "addr = 0x%x address = 0x%x %i returndata = 0x%x " % (self._addr, address, address, returndata)
        return returndata


	


