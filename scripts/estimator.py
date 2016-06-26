#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from teamshield.srv import GetMeasurement
from probQuad import *
import StringIO
import pickle


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
            self.q_tree.add_measurement_xyz(response.measurement, location_xyz)

            # Send map
            qtmap_msg = pickle.dumps(self.q_tree)
            self.map_pub.publish(qtmap_msg)
            
    def qtmap_callback(self, qtmap_msg):
        qtmap = pickle.loads(qtmap_msg.data)
        #self.q_tree.merge_trees(qtmap) uncomment this when merge_trees is done

    def __init__(self):

        rospy.init_node('estimator')
        
        self.q_tree = Root(bbox=(0, 500, 0, 500))

        self.loc_sub = rospy.Subscriber("current_location", Point, self.current_location_callback)
        self.map_sub = rospy.Subscriber("/qtmap", String, self.qtmap_callback)
        self.map_pub = rospy.Publisher('/qtmap', String, queue_size=1)

        self.get_measurement = rospy.ServiceProxy('/get_measurement', GetMeasurement)

       

      

if __name__ == '__main__':
    Estimator()
    rospy.spin()
