#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from probQuad import *
import pickle

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
      
      #what level am I on?
      
      #what other uav's are in my level?

      goal = Point()
      goal.x = 100.0;
      goal.y = 100.0;
      goal.z = 100.0;
      self.goal_pub.publish(goal)
      
  def qtmap_callback(self, qtmap_msg):
      self.q_tree = pickle.loads(qtmap_msg.data)
      
  def __init__(self):

      rospy.init_node('estimator')
      self.swarm_size = rospy.get_param('/swarm_size')
      self.map_z_min = rospy.get_param('/min_z')
      self.map_z_min = rospy.get_param('/max_z')      
      
      agents_left = self.swarm_size
      
      curr_levels = 0;
      agents_for_next_level = 1
      while (agents_left > 0){
        #alocate an agent to a level
        agents_left--
        agents_for_next_level--        
        
        if (agents_for_next_level == 0){
          curr_levels++
          agents_for_next_level = (curr_levels)*(curr_levels)
          
          #special case for first level
          if (agents_for_next_level == 1){
            agents_for_next_level++
          }
        }
      }
      self.num_levels = curr_levels
      
      
      self.q_tree = Root(bbox=(0, 500, 0, 500))
      
      self.pose_sub = []
      for i in range(0, self.swarm_size):
        uav_location.append(Point())
        callback = create_uav_location_callback(i)
        self.pose_sub.append(rospy.Subscriber("/uav" + str(i) + "/current_location", Point, callback))
        
      self.current_location_sub = rospy.Subscriber("current_location", Point, self.current_location_callback)
      
      self.map_sub = rospy.Subscriber("my_qtmap", String, self.qtmap_callback)
      
      self.goal_pub = rospy.Publisher('goal', Point, queue_size=1)

if __name__ == '__main__':
    Planner()
    rospy.spin()
