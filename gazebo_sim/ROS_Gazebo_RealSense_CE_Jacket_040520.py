"""This script receives a depth image from the pico flexx camera from a raspberry
pi 3b+ in an numpy format and publishes it to a ros topic in an opencv format
"""

import argparse
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import math
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import PIL.Image
import requests
from io import BytesIO
from IPython.display import display
import sys
from geometry_msgs.msg import Pose

import rospy
from geometry_msgs.msg import Pose
import gazebo_msgs
import gazebo_ros
import gazebo_plugins
from gazebo_msgs.msg import ModelStates

from scipy.spatial.transform import Rotation as R
import transformations as tf
from pyquaternion import Quaternion


#img_counter = 0

def callback_depth(data):
    
    bridge = CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough') #desired_encoding='passthrough')#'mono8')
    (rows,cols) = cv_image_depth.shape    #depth
    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image_depth, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imshow("Depth_image",cv_image_depth)#cv_image_depth) # depth_colormap) #
    key=cv2.waitKey(1)
    
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        print('QQQQQQQQQ button pressed')
          

    elif key == 32:
        # SPACE pressed
        print('SPACE button pressed')
        pointcloud=[]
        #print("Number of columns",cols)
        #print("Number of rows",rows)
        #print(cv_image_depth.shape)
        for i in range (cols-1):
            for j in range (rows-1):
                depth_value=cv_image_depth[j,i] #pixel depth value
                if math.isnan(depth_value) == False:
                    pointcloud.append(pixel_to_point(depth_value,i,j))
                    #print(cv_image_depth[i])          
        pointcloud = np.array(pointcloud)
        pointcloud_name = "pointcloud_03.txt"
        #pointcloud_name = "pointcloud_{}.txt".format(img_counter)
        path='/home/acoubuntu/Documents/P6/ROS_subscriber/'
        np.savetxt(path+pointcloud_name, pointcloud)
        print("Point cloud written!".format(pointcloud_name))
        print("{} written!".format(pointcloud_name))
        #img_counter += 1 

def pixel_to_point(z,x,y):
    #z = depth.get_distance(x, y)   # read from depth numpy pixel values
    x = z * (x - 320.5) / 554.254
    y = z * (y - 240.5) / 554.254
    return(np.array([x, y, z]))

def callback_camera_pose(msg):
    model_name = msg.name
    model_pose=msg.pose

    for i in range (0,3):
        model_name[i]
        if model_name[i]=='realsense_camera':
            pose_cam_x=model_pose[i].position.x
            pose_cam_y=model_pose[i].position.y
            pose_cam_z=model_pose[i].position.z   
            orient_cam_x=model_pose[i].orientation.x
            orient_cam_y=model_pose[i].orientation.y
            orient_cam_z=model_pose[i].orientation.z
            orient_cam_w=model_pose[i].orientation.w

        elif model_name[i]=='ce_jacket':
            pose_jac_x=model_pose[i].position.x
            pose_jac_y=model_pose[i].position.y
            pose_jac_z=model_pose[i].position.z   
            orient_jac_x=model_pose[i].orientation.x
            orient_jac_y=model_pose[i].orientation.y
            orient_jac_z=model_pose[i].orientation.z
            orient_jac_w=model_pose[i].orientation.w

    
    Transl_cam=tf.translation_matrix(np.array((pose_cam_x,pose_cam_y,pose_cam_z)))
    Transl_jac=tf.translation_matrix(np.array((pose_jac_x,pose_jac_y,pose_jac_z)))
    Rot_cam = np.array([orient_cam_x, orient_cam_y, orient_cam_z, orient_cam_w])
    


def listener():
    rospy.init_node('ROS_subscriber_sensor_msg')
    r = rospy.Rate(10) 
    #subscribe to the sensor messages from /camera/depth/image_raw
    rospy.Subscriber('/camera/depth/image_raw', Image, callback_depth, queue_size=1)
    #rospy.Subscriber('/gazebo/model_states', ModelStates, callback_camera_pose)
    #img_counter = 0
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    
    listener()
    