#!/usr/bin/env python3

import pyttsx3
import threading
import numpy as np
import time
import cv2
import math
from scipy import interpolate
import random
import pyrealsense2 as rs
from irohms_iql.msg import Elements
import speech_recognition as sr
import subprocess
from gtts import gTTS
import rospy
import re
import os
from constants import *
from std_msgs.msg import String
#import tf


kuka.init_robot()
time.sleep(15)
# kuka.publish_grip_cmd(gripper_reset)
rospy.loginfo("Robot ready!!")
#kuka.publish_pose(pre_grasping_pose)
#rospy.loginfo("Position reached!!")
########
x,y,z=-0.55,0.0,0.4
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("pre grasping position reached!!!")
########
x,y,z=-0.55,0.0,0.28
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Grasping position reached!!")
########
kuka.publish_grip_cmd(gripper_close)
time.sleep(2)
########
x,y,z=-0.55,0.0,0.4
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Moving to human!!")
time.sleep(0.5)
########
x,y,z= 0.001, -0.55, 0.29
euler_angles = [ 3.14159, -1.5707,0 ]
#euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Human operation position reached!!")
time.sleep(5)
########
x,y,z= -0.55,-0.25, 0.4
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Putting object to target position!!")
time.sleep(0.5)
########
x,y,z= -0.55, -0.25, 0.31
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Human operation position reached!!")
########
kuka.publish_grip_cmd(gripper_open)
########
x,y,z= -0.55, -0.25, 0.4
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Human operation position reached!!")
print('task finished!!')
########
kuka.publish_pose(waiting_pose)
rospy.loginfo("pre grasping position reached!!!")
########
# kuka.publish_grip_cmd(gripper_open)

# kuka.home()
########

        
