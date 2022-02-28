# Author: Trevor Sherrard
# Since: Feburary 24, 2022
# Project: UAV Based Post Obstruction Assesment Captsone Project
# Purpose: This file contains the implementation of a ROS node that
#          attempts to extract object detections from a given frame
#          taken from the drone in airsim

import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image

class AirsimObjectDetection:
    def __init__(self):
        self.image_pub_topic_pat = "airsim_based_drone_object_detection/vn_{}/cam_{}"

    def start_node(self):
        pass

    def do_object_detection():
        pass
    
    def run_node(self):
        pass

    def pub_images_and_data(self):
        pass

if(__name__ == "__main__"):
    airsim_obj_det = AirsimObjectDetection()

    start_status = airsim_obj_det.start_node()

    if(start_status):
        airsim_obj_det.run_node()
    else:
        rospy.logerr("could not start node!")
