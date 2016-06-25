#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

def current_location_callback(current_location):
    rospy.loginfo("I am at %f %f %f", current_location.x, current_location.y, current_location.z);
    
def estimator():

    rospy.init_node('estimator')

    rospy.Subscriber("current_location", Point, current_location_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    estimator()
