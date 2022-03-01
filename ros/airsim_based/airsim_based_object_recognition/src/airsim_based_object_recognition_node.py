#!/usr/bin/env python3

# Author: Trevor Sherrard
# Since: Feburary 24, 2022
# Project: UAV Based Post Obstruction Assesment Captsone Project
# Purpose: This file contains the implementation of a ROS node that
#          attempts to extract object detections from a given frame
#          taken from the drone in airsim

import numpy as np
import cv2
import rospy
import airsim

from sensor_msgs.msg import Image

class AirsimObjectDetection:
    def __init__(self):
        self.image_pub_topic_pat = "airsim_based_drone_object_detection/vn_{}/cam_{}"
        self.camera_name_list = list()
        self.mesh_match_regex = ""

        # set image type and detection radius
        self.image_type = airsim.ImageType.Scene
        self.detection_radius = 200*100 #cm

    def start_node(self):
        # start ROS node
        rospy.init_node("airsim_based_drone_detection_node")
        rospy.loginfo("started airsim_based_drone_detection_node!")

        # get ROS params from launch file
        self.mesh_match_regex = rospy.get_param("~mesh_match_regex")
        camera_name_list_str = rospy.get_param("~camera_name_list")

        # convert drone list string representation to string
        self.camera_name_list = camera_name_list_str.strip("][").split(", ")

        # need list to have at least on element inside.
        if(len(self.camera_name_list) == 0 or self.camera_name_list == None):
            rospy.logerr("need at least one camera name to use for object detection!")
            return False

        # need a non-empty regex to match meshes
        if(len(self.mesh_match_regex) == 0):
            rospy.logerr("Need non-empty regex for mesh matching!")
            return False

        # create airsim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # loop through all camera names and clear previus detection filter
        # settings
        for camera_name in self.camera_name_list:
            self.client.simClearDetectionMeshNames(camera_name, self.image_type)

        # loop through all camera names and set detection filter radius
        # and mesh filer regex
        for camera_name in self.camera_name_list:
            self.client.simSetDetectionFilterRadius(camera_name, self.image_type, self.detection_radius)
            self.client.simAddDetectionFilterMeshName(camera_name, self.image_type, self.mesh_match_regex)

        # if we got this far, return true
        return True

    def do_object_detection(self):
        while(not rospy.is_shutdown()):
            # loop through each provided camera and try to get 
            # detections
            for camera_name in self.camera_name_list:
                detections = self.client.simGetDetections(camera_name, self.image_type)
                print(detections)

    def pub_images_and_data(self):
        pass

if(__name__ == "__main__"):
    # create object instance
    airsim_obj_det = AirsimObjectDetection()

    # try to start node
    start_status = airsim_obj_det.start_node()

    # do simulation based object detection if
    # node could start
    if(start_status):
        airsim_obj_det.do_object_detection()
    else:
        rospy.logerr("could not start node!")
