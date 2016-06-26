#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

class Planner:
  def current_location_callback(self, my_location):
      rospy.loginfo("I am at %f %f %f", my_location.x, my_location.y, my_location.z);

      goal = Point()
      goal.x = 100.0;
      goal.y = 100.0;
      goal.z = 100.0;
      self.goal_pub.publish(goal)
      
  def __init__(self):

      rospy.init_node('estimator')

      rospy.Subscriber("current_location", Point, self.current_location_callback)
      
      self.goal_pub = rospy.Publisher('goal', Point, queue_size=1)

if __name__ == '__main__':
    Planner()
    rospy.spin()
