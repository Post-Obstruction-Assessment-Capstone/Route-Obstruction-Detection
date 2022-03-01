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
import copy
import cv_bridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from airsim_based_object_recognition.msg import airsim_obstacle_msg, airsim_obstacle_list_msg

class AirsimObjectDetection:
    def __init__(self):
        self.drawn_image_pub_topic_pat = "airsim_based_drone_object_detection/drawn_image/{}"
        self.detected_obstacle_list_pat = "airsim_based_drone_object_detection/detection_list/{}"
        self.camera_name_list = list()
        self.mesh_match_regex = ""

        # create dictionaries to hold publisher objects for
        # drawn over images and detection list messages
        self.drawn_image_pub_dict = dict()
        self.detected_obstacle_list_pub_dict = dict()

        # set image type and detection radius
        self.image_type = airsim.ImageType.Scene
        self.detection_radius = 2000*1000 #cm

        # create cv_bridge
        self.bridge = cv_bridge.CvBridge()

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

        # create publishers
        for camera_name in self.camera_name_list:
            temp_drawn_image_pub = rospy.Publisher(self.drawn_image_pub_topic_pat.format(camera_name),
                                                   Image, queue_size=1)
            temp_obstacle_list_pub = rospy.Publisher(self.detected_obstacle_list_pat.format(camera_name),
                                                     airsim_obstacle_list_msg, queue_size=1)
            self.drawn_image_pub_dict[camera_name] = temp_drawn_image_pub
            self.detected_obstacle_list_pub_dict[camera_name] = temp_obstacle_list_pub

        rospy.loginfo("Created publishers!")

        # create airsim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        rospy.loginfo("Connected to airsim!")

        # loop through all camera names and clear previus detection filter
        # settings
        for camera_name in self.camera_name_list:
            self.client.simClearDetectionMeshNames(camera_name, self.image_type)
        rospy.loginfo("Cleared old detection mesh name regex pattern for cameras: {}".format(self.camera_name_list))

        # loop through all camera names and set detection filter radius
        # and mesh filer regex
        for camera_name in self.camera_name_list:
            self.client.simSetDetectionFilterRadius(camera_name, self.image_type, self.detection_radius)
            self.client.simAddDetectionFilterMeshName(camera_name, self.image_type, self.mesh_match_regex)
        rospy.loginfo("Set new detection mesh name regex pattern for cameras: {}".format(self.camera_name_list))

        # if we got this far, return true
        return True

    def do_object_detection(self):
        while(not rospy.is_shutdown()):
            # loop through each provided camera and try to get 
            # detections
            for camera_name in self.camera_name_list:
                # try and grab an image
                responses = self.client.simGetImages([airsim.ImageRequest(camera_name, self.image_type, False, False)])
                raw_image = responses[0]

                # get numpy array
                img1d = np.fromstring(raw_image.image_data_uint8, dtype=np.uint8)

                # reshape array to 4 channel image array H X W X 4
                img_rgb = img1d.reshape(raw_image.height, raw_image.width, 3)

                # try and grab detections
                detections = self.client.simGetDetections(camera_name, self.image_type)

                # send detections and image to be published as custom message.
                self.pub_images_and_data(camera_name, img_rgb, detections)

    def pub_images_and_data(self, camera_name, image_dec, detections):
        # if we have detections...
        if(len(detections) != 0):
            # make a copy of the image
            image_dec_drawn_over = copy.deepcopy(image_dec)

            # create a airsim_obstacle_msg_list object
            obstacle_list_msg = airsim_obstacle_list_msg()
            obstacle_list_msg.obstacle_list = list()

            # loop through them and extract information
            for item in detections:
                # create a single airsim_obstacle_msg object
                obstacle_msg = airsim_obstacle_msg()

                # populate fields

                # camera and item name
                obstacle_msg.obstacle_name = item.name
                obstacle_msg.camera_name = camera_name

                # bounding box coordinates
                item_2D_bbox = item.box2D
                obstacle_msg.bbox_TL_row = int(item_2D_bbox.min.x_val)
                obstacle_msg.bbox_TL_col = int(item_2D_bbox.min.y_val)
                obstacle_msg.bbox_BR_row = int(item_2D_bbox.max.x_val)
                obstacle_msg.bbox_BR_col = int(item_2D_bbox.max.y_val)

                # relative pose of obstacle
                temp_relative_pose = Pose()
                temp_relative_pose.position.x = item.relative_pose.position.x_val
                temp_relative_pose.position.y = item.relative_pose.position.y_val
                temp_relative_pose.position.z = item.relative_pose.position.z_val
                temp_relative_pose.orientation.x = item.relative_pose.orientation.x_val
                temp_relative_pose.orientation.y = item.relative_pose.orientation.y_val
                temp_relative_pose.orientation.z = item.relative_pose.orientation.z_val
                temp_relative_pose.orientation.w = item.relative_pose.orientation.w_val
                obstacle_msg.relative_pose = temp_relative_pose

                # add detected obstacle to list
                obstacle_list_msg.obstacle_list.append(obstacle_msg)

                # convert raw image to ROS image message
                img_msg = self.bridge.cv2_to_imgmsg(image_dec, encoding="bgr8")
                obstacle_list_msg.raw_image = img_msg

                # draw 2D bounding boxes over image
                cv2.rectangle(image_dec_drawn_over, (int(item_2D_bbox.min.x_val), int(item_2D_bbox.min.y_val)),
                              (int(item_2D_bbox.max.x_val), int(item_2D_bbox.max.y_val)), (255, 0, 0), 2)

                # publish detected object list
                self.detected_obstacle_list_pub_dict[camera_name].publish(obstacle_list_msg)

                # publish drawn over image
                drawn_img_msg = self.bridge.cv2_to_imgmsg(image_dec_drawn_over, encoding="bgr8")
                self.drawn_image_pub_dict[camera_name].publish(drawn_img_msg)

if(__name__ == "__main__"):
    # create object instance
    airsim_obj_det = AirsimObjectDetection()

    # try to start node
    start_status = airsim_obj_det.start_node()

    # do simulation based object detection if
    # node could start
    if(start_status):
        rospy.loginfo("Starting object detection loop!")
        airsim_obj_det.do_object_detection()
    else:
        rospy.logerr("could not start node!")
