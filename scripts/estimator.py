#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from teamshield.srv import GetMeasurement
from probQuad import *

class Estimator:
    def current_location_callback(self, current_location):
        rospy.loginfo("I am at %f %f %f", current_location.x, current_location.y, current_location.z);


        in_new_cell = True # logic required here
        if in_new_cell:
            try:
                response = self.get_measurement(current_location.x, current_location.y, current_location.z)
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s", e);
                return

            location_xyz = (current_location.x, current_location.y, current_location.z)
            self.q_tree.add_measurement(response.measurement, location_xyz)


    def __init__(self):

        rospy.init_node('estimator')

        rospy.Subscriber("current_location", Point, self.current_location_callback)

        self.get_measurement = rospy.ServiceProxy('/get_measurement', GetMeasurement)

        self.q_tree = Root(bbox=(0, 500, 0, 500))
      

if __name__ == '__main__':
    Estimator()
    rospy.spin()
