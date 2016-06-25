#!/usr/bin/env python


import rospy
import numpy as np
from numpy import genfromtxt
from std_msgs.msg import Float32
from teamshield.srv import GetMeasurement, GetMeasurementResponse


class WorldLoader:
    def __init__(self):
        filename = rospy.get_param("/worldfile", "/home/kenny/world.csv")
        # Assumption: Data loaded row-major
        self.world_array = genfromtxt(filename, delimiter=",")
        self.meas_srv = rospy.Service("get_measurement", GetMeasurement, self.get_measurement)
        print(self.world_array)

    def get_measurement(self, req):
        res = 0.0
        if not (req.x < 0 or np.size(self.world_array, axis=1) < req.x or \
            req.y < 0 or np.size(self.world_array, axis=0) < req.y):
            res = self.world_array[req.x, req.y]
        else:
            rospy.logwarn("Location out of bounds requested")
            res = 0.0
        return GetMeasurementResponse(res)

if __name__ == "__main__":
    rospy.init_node("world_loader")
    world_loader = WorldLoader()
    rospy.spin()

