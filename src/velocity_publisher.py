#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelPublisher(object):
    def __init__(self):
        self.topic_name = "/cmd_vel"
        self.velocity_publisher = rospy.Publisher(self.topic_name, Twist, queue_size= 10)
        self.linear_velocity = 0.5
        self.angular_velocity = 0.5

    def move_robot(self, message):
        velocity_message = Twist()
        if message == "forward":
            rospy.loginfo("Moving forward...")
            velocity_message.linear.x = self.linear_velocity
            velocity_message.angular.z = 0.0

        elif message == "backward":
            rospy.loginfo("Moving backwards...")
            velocity_message.linear.x = -self.linear_velocity
            velocity_message.angular.z = 0.0

        elif message == "right":
            rospy.loginfo("Turning right...")
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = -self.angular_velocity

        elif message == "left":
            rospy.loginfo("Turning left...")
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = self.angular_velocity

        elif message == "stop":
            rospy.loginfo("Stopping robot...")
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0

        else:
            pass

        self.velocity_publisher.publish(velocity_message)

if __name__ == "__main__":
    rospy.loginfo("velocity_publisher", anonymous = True)
    loop_rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        global ctrl_c
        rospy.loginfo("Shuttin down...")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    CmdVelPublisher_object = CmdVelPublisher()

    while not ctrl_c:
        try:
            CmdVelPublisher_object.move_robot("forward")
            loop_rate.sleep()

        except rospy.ROSInterruptException:
            pass