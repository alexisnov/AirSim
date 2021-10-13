import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

value = client.getJSBSimProperty("position/h-sl-meters", "Rocket")
print("altitude: %f" % value)

client.setJSBSimProperty("position/h-sl-meters", 2, "Rocket")

value = client.getJSBSimProperty("position/h-sl-meters", "Rocket")
print("altitude: %f" % value)

value = client.getJSBSimProperty("simulation/terminate", "Rocket")
print("terminate: %f" % value)

client.setJSBSimProperty("simulation/terminate", 1, "Rocket")

value = client.getJSBSimProperty("simulation/terminate", "Rocket")
print("terminate: %f" % value)

#client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
