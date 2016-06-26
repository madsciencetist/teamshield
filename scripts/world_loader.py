#!/usr/bin/env python


import rospy
import numpy as np
import os
import math
from numpy import genfromtxt
from std_msgs.msg import Float32
from teamshield.srv import GetMeasurement, GetMeasurementResponse
from random import randint

def in_range(val, min, max):
    return val >= min and val <= max


class WorldLoader:
    def __init__(self):
        filename = os.path.abspath(rospy.get_param("/filename", "files/occlusion.csv"))
        self.min_z = rospy.get_param("/min_z", 100.)
        self.max_z = rospy.get_param("/max_z", 600.)
        self.max_n = rospy.get_param("/max_n", 128)
        self.camera_fov = rospy.get_param("/camera_fov", 30.)
        self.m_per_box = rospy.get_param("/m_per_box", 5.)
        self.bound_dist = self.max_n * self.m_per_box
        # Assumption: Data loaded row-major
        self.world_env = genfromtxt(filename, delimiter=",")
        self.target = self.generate_targets(8)
        self.meas_srv = rospy.Service("/get_measurement", GetMeasurement, self.get_measurement)

    def get_measurement(self, req):
        res = 0.0
        if in_range(req.x, 0, np.size(self.world_env, axis=1) * self.m_per_box) and \
           in_range(req.y, 0, np.size(self.world_env, axis=0) * self.m_per_box):
            # x, y, z is in meters. need to correlate x, y meter to cell
            res = self.weighted_probability(req.x / self.m_per_box, req.y / self.m_per_box, req.z)
            rospy.loginfo("{} - RAW: {}, empty prob: {}".format((req.y * self.max_n + req.x), self.world_env[req.x, req.y], res))
        else:
            rospy.logwarn("Location out of bounds requested")
            res = 1.0
        return GetMeasurementResponse(res)

    def weighted_probability(self, x, y, z):
        val = 0.0
        # check if our altitude
        # if below min_z, assume we can see everything
        # if above max_z, assume we cannot see anything
        # otherwise, calculate
        if z > self.min_z and z <= self.max_z:
            # determine patch of grid that is visible
            max_box_range = abs(z * math.tan(math.radians(self.camera_fov)))
            max_box_count = int(max_box_range / self.m_per_box)
            b_min_x = int(max(x - max_box_count, 0))
            b_max_x = int(min(x + max_box_count, self.max_n))
            b_min_y = int(max(y - max_box_count, 0))
            b_max_y = int(min(y + max_box_count, self.max_n))
            patch_loc = []
            for yy in range(b_min_y, b_max_y):
                for xx in range(b_min_x, b_max_x):
                    patch_loc.append((xx, yy))
            # extract occlusion for path and find max value
            val = max([self.occl(self.world_env[xxx, yyy], z) for (xxx, yyy) in patch_loc])
            val = val if (int(y * self.max_n + x) in self.target) else (1.0 - val)
        elif z > self.max_z:
            val = 1.0
        else:
            val = 0.0
        return val

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
