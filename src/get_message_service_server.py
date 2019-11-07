#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from laser_subscriber import LaserReader

class GetDirectionMessage(object):
    def __init__(self):
        service_name = "/get_direction_message_service"
        self.service_server = rospy.Service(service_name, Trigger, self.service_callback)
        self.LaserReader_object = LaserReader()
        self.detection_dict = {}
        self.threshold = 0.6

    def service_callback(self, request):
        self.detection_dict = self.LaserReader_object.crash_detector()
        response = TriggerResponse()
        response.success = False
        response.message = self.direction_to_move()
        return response

    #def obstacle_close(self):
     #   data = self.LaserReader_object.get_laser_data()
      #  for i in range(len(data)):
       #     if data[i] < self.threshold:
        #        return True
        #return False

    def direction_to_move(self):

        if self.detection_dict["front"] > self.threshold and self.detection_dict["front_right"] > self.threshold and self.detection_dict["front_left"] > self.threshold:
            rospy.loginfo("[RIGHT: {}, FRONT RIGHT: {}, FRONT: {}, FRONT LEFT: {}, LEFT: {}]".format(str(self.detection_dict["right"]), str(self.detection_dict["front_right"]), str(self.detection_dict["front"]), str(self.detection_dict["front_left"]), str(self.detection_dict["left"])))
            message = "forward"

        else:
            if self.detection_dict["front_right"] < self.threshold or self.detection_dict["right"] < self.threshold:
                rospy.logwarn("[FRONT RIGHT: {}, RIGHT: {}]".format(str(self.detection_dict["front_right"]), str(self.detection_dict["right"])))
                rospy.loginfo("[FRONT: {}, FRONT LEFT: {}, LEFT: {}]".format(str(self.detection_dict["front"]), str(self.detection_dict["front_left"]), str(self.detection_dict["left"])))
                message = "left"

            else:
                if self.detection_dict["front_left"] < self.threshold or self.detection_dict["left"] < self.threshold:
                    rospy.logwarn("[FRONT LEFT: {}, LEFT: {}]".format(str(self.detection_dict["front_left"]), str(self.detection_dict["left"])))
                    rospy.loginfo("[RIGHT: {}, FRONT RIGHT: {}, FRONT: {}]".format(str(self.detection_dict["right"]), str(self.detection_dict["front_right"]),str(self.detection_dict["front"])))
                    message = "right"

                else:
                    if self.detection_dict["right"] > self.threshold + 0.5:
                        message = "right"

                    else:
                        if self.detection_dict["left"] > self.threshold + 0.5:
                            message = "left"

                        else:
                            message = "no obstacle"

        return message

if __name__ == "__main__":
    rospy.init_node("get_direction_message_service", anonymous = True)
    GetDirectionMessage_object = GetDirectionMessage()
    rospy.spin()
