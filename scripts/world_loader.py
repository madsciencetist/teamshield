#!/usr/bin/env python


import rospy
import numpy as np
from numpy import genfromtxt
from std_msgs.msg import Float32
from teamshield.srv import GetMeasurement, GetMeasurementResponse


class WorldLoader:
    def __init__(self):
        self.min_z = rospy.get_param("/min_z", 0)       # altitude (m)
        self.max_z = rospy.get_param("/max_z", 9800)    # altitude (m)
        self.layer_num = rospy.get_param("/layer_num", 5)
        self.axis_size = rospy.get_param("/axis_size", 128)
        # Assumption: Data loaded row-major
        self.world_array = self.generate_world(max_n=self.axis_size)
        self.meas_srv = rospy.Service("get_measurement", GetMeasurement, self.get_measurement)

    def get_measurement(self, req):
        res = 0.0
        if not (req.x < 0 or np.size(self.world_array, axis=1) < req.x or \
            req.y < 0 or np.size(self.world_array, axis=0) < req.y):
            res = self.occl(self.world_array[req.x, req.y], req.z)
        else:
            rospy.logwarn("Location out of bounds requested")
            res = 0.0
        return GetMeasurementResponse(res)

    def occl(self, cell_val, z):
        return (cell_val * (z - self.min_z)) / (self.max_z - self.min_z)

    def generate_world(self, num_centers=8, max_n=128):
        X = np.zeros((max_n * max_n, 2))
        centers = np.random.rand(num_centers, 2)
        widths = np.random.rand(num_centers, 1) / 48
        occlusion = np.zeros(max_n * max_n)
        result = np.zeros((max_n, max_n))
        for i in range(max_n):
            for j in range(max_n):
                n = i * max_n + j
                for k in range(num_centers):
                    occlusion[n] += np.exp(np.power(-np.linalg.norm(centers[k,:]-X[n,:]), 2)/widths[k])

        max_val = 1.3 * max(occlusion)

        for i in range(max_n):
            for j in range(max_n):
                n = i * max_n + j
                tmp = occlusion[n]/max_val;
                result[i, j] = tmp
        return result

if __name__ == "__main__":
    rospy.init_node("world_loader")
    world_loader = WorldLoader()
    rospy.spin()
