#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from teamshield.srv import GetMeasurement

class Estimator:
  def current_location_callback(self, current_location):
      rospy.loginfo("I am at %f %f %f", current_location.x, current_location.y, current_location.z);

      
      in_new_cell = True # logic required here
      if in_new_cell:
        try:
          response = self.get_measurement(current_location.x, current_location.y, current_location.z);
          measurement = response.measurement
          rospy.loginfo("Measured %f", measurement);
        except rospy.ServiceException, e:
          rospy.loginfo("Service call failed: %s", e);
      
  def __init__(self):

      rospy.init_node('estimator')

      rospy.Subscriber("current_location", Point, self.current_location_callback)
      
      self.get_measurement = rospy.ServiceProxy('/get_measurement', GetMeasurement)

if __name__ == '__main__':
    Estimator()
    rospy.spin()
