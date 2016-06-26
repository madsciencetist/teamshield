#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

uav_location = []

def create_uav_location_callback(i):
    def callback(point):
        uav_location[i] = point
    return callback

class Planner:
  def current_location_callback(self, my_location):
      rospy.loginfo("I am at %f %f %f", my_location.x, my_location.y, my_location.z);
      
      for i in range(0, self.swarm_size):
        rospy.loginfo("UAV %d is at %f %f %f", i, uav_location[i].x, uav_location[i].y, uav_location[i].z)

      goal = Point()
      goal.x = 100.0;
      goal.y = 100.0;
      goal.z = 100.0;
      self.goal_pub.publish(goal)
      
  def uav1_location_call(self, msg):
      print(msg)
      
  def __init__(self):

      rospy.init_node('estimator')
      
      self.current_location_sub = rospy.Subscriber("current_location", Point, self.current_location_callback)
      
      self.swarm_size = rospy.get_param('/swarm_size')
      
      self.pose_sub = []
      
      for i in range(0, self.swarm_size):
        uav_location.append(Point())
        callback = create_uav_location_callback(i)
        self.pose_sub.append(rospy.Subscriber("/uav" + str(i) + "/current_location", Point, callback))
      
      self.goal_pub = rospy.Publisher('goal', Point, queue_size=1)

if __name__ == '__main__':
    Planner()
    rospy.spin()
