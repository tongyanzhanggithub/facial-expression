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
time.sleep(5)
rospy.loginfo("Robot ready!!")
#kuka.publish_pose(pre_grasping_pose)
#rospy.loginfo("Position reached!!")
########
x,y,z=0.55,0.0,0.5
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Position reached!!")
########
x,y,z=0.55,0.0,0.3
euler_angles=kuka.get_angles([x,y,z],0.0)
new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
kuka.publish_pose(new_pose_msg)
rospy.loginfo("Position reached!!")
time.sleep(5)
########
########

            
        
