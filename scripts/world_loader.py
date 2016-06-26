#!/usr/bin/env python


import rospy
import numpy as np
import os
from numpy import genfromtxt
from std_msgs.msg import Float32
from teamshield.srv import GetMeasurement, GetMeasurementResponse
from random import randint

def in_range(val, min, max):
    return val >= min and val <= max


class WorldLoader:
    def __init__(self):
        filename = os.path.abspath(rospy.get_param("/filename", "files/occlusion.csv"))
        self.min_z = rospy.get_param("/min_z", 1500)       # altitude (m)
        self.max_z = rospy.get_param("/max_z", 9800)    # altitude (m)
        self.layer_num = rospy.get_param("/layer_num", 6)
        self.max_n = rospy.get_param("/max_n", 128)
        # Assumption: Data loaded row-major
        # self.world_environment = self.generate_environment()
        self.world_environment = genfromtxt(filename, delimiter=",")
        self.target = self.generate_targets(8)
        print(self.world_environment.shape)
        print(self.world_environment)
        print(self.target)
        self.meas_srv = rospy.Service("get_measurement", GetMeasurement, self.get_measurement)

    def get_measurement(self, req):
        res = 0.0
        if in_range(req.x, 0, np.size(self.world_environment, axis=1)) and \
           in_range(req.y, 0, np.size(self.world_environment, axis=0)) and \
           in_range(req.z, self.min_z, self.max_z):
            res = self.weighted_probability(req.x, req.y, req.z)
            rospy.loginfo("{} - RAW: {}, empty prob: {}".format((req.y * self.max_n + req.x), self.world_environment[req.x, req.y], res))
        else:
            rospy.logwarn("Location out of bounds requested")
            res = 0.0
        return GetMeasurementResponse(res)

    def weighted_probability(self, x, y, z):
        val = self.occl(self.world_environment[x, y], z)
        return val if (int(y * self.max_n + x) in self.target) else (1.0 - val)

    def occl(self, cell_val, z):
        return (cell_val * (z - self.min_z)) / (self.max_z - self.min_z)

    def generate_environment(self, num_centers=8):
        X = np.zeros((self.max_n * self.max_n, 2))
        centers = np.random.rand(num_centers, 2)
        widths = np.random.rand(num_centers, 1) / 48
        occlusion = np.zeros(self.max_n * self.max_n)
        result = np.zeros((self.max_n, self.max_n))
        for i in range(self.max_n):
            for j in range(self.max_n):
                n = i * self.max_n + j
                for k in range(num_centers):
                    occlusion[n] += np.exp(np.power(-np.linalg.norm(centers[k,:]-X[n,:]), 2)/widths[k])

        max_val = 1.3 * max(occlusion)

        for i in range(self.max_n):
            for j in range(self.max_n):
                n = i * self.max_n + j
                tmp = occlusion[n]/max_val;
                result[i, j] = tmp
        return result

    def generate_targets(self, targets):
        total_cells = self.max_n * self.max_n
        return [randint(0, total_cells) for i in range(targets)]


if __name__ == "__main__":
    rospy.init_node("world_loader")
    world_loader = WorldLoader()
    rospy.spin()
