#! /usr/bin/env python

import rospy
import actionlib
from turtlebot_maze_solving.msg import RecordOdomAction, RecordOdomFeedback, RecordOdomGoal, RecordOdomResult
from std_srvs.srv import Trigger, TriggerRequest
from odom_subscriber import OdomReader
from odom_analysis import OdomAnalysis
from velocity_publisher import CmdVelPublisher
import time

class ControlTurtle(object):
    def __init__(self, goal_distance):
        self.goal_distance = goal_distance
        self.OdomAnalysis_object = OdomAnalysis()
        self.init_direction_service_client()
        self.init_record_odom_action_client()
        self.init_move_turtle_publisher()

    def init_direction_service_client(self):
        self.service_name = "/get_direction_message_service"
        rospy.loginfo("Waiting for service '/get_direction_message_service'...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Found service '/get_direction_message_service'...")
        self.service_client = rospy.ServiceProxy(self.service_name, Trigger)
        self.service_request = TriggerRequest()

    def make_direction_request(self):
        result = self.service_client(self.service_request)
        return result.message

    def init_record_odom_action_client(self):
        self.action_name = "/record_odom_action"
        self.action_client = actionlib.SimpleActionClient(self.action_name, RecordOdomAction)
        rospy.loginfo("Waiting for action '/record_odom_action'...")
        self.action_client.wait_for_server()
        rospy.loginfo("Found action '/record_odom_action'...")
        self.action_goal = RecordOdomGoal()

    def send_action_goal(self):
        self.action_client.send_goal(self.action_goal, feedback_cb = self.action_feedback_callback)

    def action_feedback_callback(self, feedback):
        pass

    def record_odom_finished(self):
        has_finished = self.action_client.get_state() >= 2
        return has_finished

    def get_odom_result(self):
        return self.action_client.get_result()

    def init_move_turtle_publisher(self):
        self.CmdVelPublisher_object = CmdVelPublisher()

    def move_turtle(self, direction):
        self.CmdVelPublisher_object.move_robot(direction)

    def got_out_of_maze(self, result_odom_array):
        return self.OdomAnalysis_object.goal_reached(self.goal_distance, result_odom_array)

rospy.init_node("turtlebot_maze_solving")
ControlTurtle_object = ControlTurtle(goal_distance = 7.5)

loop_rate = rospy.Rate(10)

ControlTurtle_object.send_action_goal()

ctrl_c = False
def shutdownhook():
    global ctrl_c
    rospy.loginfo("Shutting Down!")
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

try:
    while not ControlTurtle_object.record_odom_finished() and not ctrl_c:
        direction_to_move = ControlTurtle_object.make_direction_request()
        ControlTurtle_object.move_turtle(direction_to_move)
        loop_rate.sleep()

    odom_result = ControlTurtle_object.get_odom_result()
    odom_result_array = odom_result.result_odom_array

    if ControlTurtle_object.got_out_of_maze(odom_result_array):
        ControlTurtle_object.move_turtle("stop")
        rospy.logwarn("Out of maze")

    else:
        ControlTurtle_object.move_turtle("stop")
        rospy.logwarn("In Maze")

    rospy.logwarn("Turtlebot maze test finished.")


except rospy.ROSInterruptException:
    pass