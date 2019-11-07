#! /usr/bin/env python

import rospy
import actionlib
from turtlebot_maze_solving.msg import RecordOdomAction, RecordOdomResult
from odom_subscriber import OdomReader
from odom_analysis import OdomAnalysis

class RecordOdometry(object):
    def __init__(self, goal_distance):
        self.goal_distance = goal_distance
        action_name = "/record_odom_action"
        self.action_server = actionlib.SimpleActionServer(action_name, RecordOdomAction, self.goal_callback, False)
        self.OdomReader_object = OdomReader()
        self.action_server.start()
        self.result = RecordOdomResult()
        self.OdomAnalysis_object = OdomAnalysis()
        self.recording_seconds = 120

    def goal_callback(self, goal):
        success = True
        loop_rate = rospy.Rate(1)

        for i in range(self.recording_seconds):
            rospy.loginfo("Recording Odom index = {}".format(str(i)))
            if self.action_server.is_preempt_requested():
                rospy.logdebug("Goal has been cancelled/preempted")
                self.action_server.preempted()
                success = False
                break
            else:
                if not self.reached_distance_goal():
                    rospy.logdebug("Reading Odometry...")
                    self.result.result_odom_array.append(self.OdomReader_object.get_odom_data())
                else:
                    rospy.logwarn("Reached goal distance")
                    break
            loop_rate.sleep()

        if success:
            self.action_server.set_succeeded(self.result)

        self.clean_variables()

    def clean_variables(self):
        self.result = RecordOdomResult()

    def reached_distance_goal(self):
        return self.OdomAnalysis_object.goal_reached(self.goal_distance, self.result.result_odom_array)

if __name__ == "__main__":
    rospy.init_node("record_odom_action", anonymous = True)
    RecordOdometry_object = RecordOdometry(goal_distance=7.5)
    rospy.spin()