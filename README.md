# Route-Obstruction-Detection
This repository contains software that allows for the detection of objects that could be considered route blockages.
The solution presented here makes use of the built-in airsim simulator functionallity. 

## Running the Node
To run the object detection node, please run the following command in a terminal:

```commandline
roslaunch airsim_based_object_recognition airsim_based_object_recognition.launch
```

Note that this launch file contains definitions of the regex pattern of meshes to detect, as well as a
list of camera names in which objects should be detected from. Please make sure to change these parameter definitions
as needed.

## Published Data Format
The node implemented here makes use of a set of custom ROS message types to publish data. In each iteration of the 
node's update loop, the presence of meshes matching the provided regex pattern are checked for in each of the provided camera
name's fields of view. If objects are detected in a particular camera's FOV, the object's 2D bounding box, pose
relative to the camera, and mesh name are extracted. These data points are then pushed into a custom message (airsim_obstacle_msg.msg), in which the format can be
seen below:

```commandline
string camera_name
string obstacle_name
uint32 bbox_TL_col
uint32 bbox_TL_row
uint32 bbox_BR_col
uint32 bbox_BR_row
geometry_msgs/Pose relative_pose
```

An instance of this message is created for each of the objects detected in a given camera's FOV. A new custom message (airsim_obstacle_list_msg.msg)
object is then created with the following structure:

```commandline
airsim_obstacle_msg[] obstacle_list
sensor_msgs/Image raw_image
```

This list of airsim_obstacle_msg objects, along with the raw_image the obstacles were observed in are published to the topic
with pattern: 

```commandline
airsim_based_drone_object_detection/detection_list/{cam_name}. 
```

Where cam_name is the given camera name defined in airsim_based_object_recognition.launch. 

This node also publishes a copy of the given image for each camera with detected objects' bounding boxes drawn over
the image itself. These images are published to the topics with pattern: 

```commandline
airsim_based_drone_object_detection/drawn_image/{cam_name}
```

Where cam_name is the given camera name defined in airsim_based_object_recognition.launch. These images are
helpful for system debugging purposes.