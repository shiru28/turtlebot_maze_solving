#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class LaserReader(object):
    def __init__(self):
        topic_name = "/kobuki/laser/scan"
        self.laser_subscriber = rospy.Subscriber(topic_name, LaserScan, self.laser_callback)
        self.laser_message = LaserScan()

    def laser_callback(self, message):
        self.laser_message = message

    def get_laser_data(self):
        return self.laser_message

    def crash_detector(self):
        self.right = self.laser_message.ranges[0]
        self.front = self.laser_message.ranges[360]
        self.left = self.laser_message.ranges[719]
        self.front_right = self.laser_message.ranges[180]
        self.front_left = self.laser_message.ranges[540]
        regions = [self.right, self.front_right, self.front, self.front_left, self.left]

        return self.convert_to_dict(regions)

    def convert_to_dict(self, data):
        detection_dict = {}
        detection_dict = {"right": data[0],
                          "front_right" : data[1],
                          "front" : data[2],
                          "front_left": data[3],
                          "left" : data[4]}

        return detection_dict


if __name__ == "__main__":
    rospy.init_node("laser_subscriber", anonymous = True)
    ctrl_c = False
    loop_rate = rospy.Rate(10)

    def shutdownhook():
        global ctrl_c
        rospy.loginfo("Shutting Down...")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    LaserReader_object = LaserReader()
    while not ctrl_c:
        try:
            data = LaserReader_object.get_laser_data()
            LaserReader_object.crash_detector()
            loop_rate.sleep()

        except rospy.ROSInterruptException:
            pass