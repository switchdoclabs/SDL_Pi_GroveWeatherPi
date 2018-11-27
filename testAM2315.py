import time
import sys

sys.path.append('./SDL_Pi_AM2315')

import AM2315

am2315 = AM2315.AM2315()

for x in range(0,10):
    outsideHumidity, outsideTemperature, crc_check = am2315.read_humidity_temperature_crc() 
    print "temperature: %0.1f" % outsideTemperature
    print "humidity: %0.1f" % outsideHumidity
    print "crc: %s" % crc_check
    print
    time.sleep(2.0)
