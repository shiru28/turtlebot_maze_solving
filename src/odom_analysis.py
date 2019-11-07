#! /usr/bin/env python

import rospy
from odom_subscriber import OdomReader
from geometry_msgs.msg import Vector3
from math import pow, sqrt

class OdomAnalysis(object):
    def __init__(self):
        pass

    def get_distance(self, result_odom_array):
        distance = None
        if len(result_odom_array)>=2:
            start_odom = result_odom_array[0]
            end_odom = result_odom_array[len(result_odom_array)-1]

            start_position = start_odom.pose.pose.position
            end_position = end_odom.pose.pose.position

            rospy.loginfo("Start position ==> "+str(start_position))
            rospy.loginfo("End position ==> "+str(end_position))

            distance_vector = self.get_vector(start_position, end_position)
            distance = self.calculate_distance(distance_vector)
            rospy.loginfo("Distance travelled: {}".format(str(distance)))
        else:
            rospy.logerr("Odom array doesn't have minimum number of elements.")

        return distance

    def get_vector(self, p1, p2):
        vector = Vector3()
        vector.x = p1.x - p2.x
        vector.y = p1.y - p2.y
        vector.z = p1.z - p2.z
        return vector

    def calculate_distance(self,vector):
        return sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2))

    def goal_reached(self, goal_distance, result_odom_array):
        distance_travelled =  self.get_distance(result_odom_array)
        return distance_travelled > goal_distance
