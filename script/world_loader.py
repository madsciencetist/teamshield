#!/usr/bin/env python


import rospy
import numpy as np
from numpy import genfromtxt
from std_msgs.msg import Float32
from teamshield.srv import GetMeasurement


class WorldLoader:
    def __init__(self):
        filename = rospy.get_param("/worldfile", "/home/kenny/world.csv")
        self.world_array = genfromtxt(filename, delimiter=",")
        self.meas_srv = rospy.Service("get_measurement", GetMeasurement, self.get_measurement)
        print(self.world_array)

    def get_measurement(self, req):
        return GetMeasurementResponse(self.world_array[req.x, req.y])

if __name__ == "__main__":
    rospy.init_node("world_loader")
    world_loader = WorldLoader()
    rospy.spin()

