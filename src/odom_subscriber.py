#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class OdomReader(object):
    def __init__(self):
        topic_name = "/odom"
        self.odom_subscriber = rospy.Subscriber(topic_name, Odometry,self.odom_callback)
        self.odom_message = Odometry()

    def odom_callback(self, message):
        self.odom_message = message

    def get_odom_data(self):
        return self.odom_message

    
if __name__ == "__main__":
    rospy.init_node("odom_subscriber", anonymous = True)
    ctrl_c = False
    loop_rate = rospy.Rate(10)

    def shutdownhook():
        global ctrl_c
        rospy.loginfo("Shutting down...")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    OdomReader_object = OdomReader()

    while not ctrl_c:
        
        try:
            data = OdomReader_object.get_odom_data()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass