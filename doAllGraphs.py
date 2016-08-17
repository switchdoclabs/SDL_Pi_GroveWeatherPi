#
# calculate all graphs
#
# SwitchDoc Labs March 30, 2015

import sys
sys.path.append('/home/pi/SDL_Pi_GroveWeatherPi/graphs')




import TemperatureHumidityGraph 
import PowerCurrentGraph 
import PowerVoltageGraph 
import BarometerLightningGraph 

def doAllGraphs():

	BarometerLightningGraph.BarometerLightningGraph('test', 10, 0)
	TemperatureHumidityGraph.TemperatureHumidityGraph('test', 10, 0)
	PowerCurrentGraph.PowerCurrentGraph('test', 10, 0)
	PowerVoltageGraph.PowerVoltageGraph('test', 10, 0)

