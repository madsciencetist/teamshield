#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from teamshield.srv import GetMeasurement
from probQuad import *
import pickle


class Estimator:
    def uav_location_callback(self, current_location):

        try:
            response = self.get_measurement(current_location.x, current_location.y, current_location.z)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s", e);
            return

        location_xyz = (current_location.x, current_location.y, current_location.z)
        self.q_tree.add_measurement_xyz(response.measurement, location_xyz)

    def __init__(self):

        rospy.init_node('estimator')
        self.swarm_size = rospy.get_param('/swarm_size')
        
        self.q_tree = Root(bbox=(0, 500, 0, 500))

        self.get_measurement = rospy.ServiceProxy('/get_measurement', GetMeasurement)
        
        # ehhhh instead of merging maps, just share measurements
        self.pose_sub = []
        for i in range(0, self.swarm_size):
            self.pose_sub.append(rospy.Subscriber("/uav" + str(i) + "/current_location", Point, self.uav_location_callback))



if __name__ == '__main__':
    Estimator()
    rospy.spin()
