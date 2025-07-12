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
import re
import os
from constants import *
from std_msgs.msg import String
#import tf
from logzero import logger, logfile
import time

distances = [0.25, 0.30, 0.35]
heights = [-0.10, 0, 0.10]
angles = [0, -3.14159/2]

TABLE_HEIGHT = 0.9
EXECUTER_HEIGHT = 0.24

PI = 3.14159
def prepare(debug):
	if debug:
		return 'test', '168'
	else:
		pass
	kuka.init_robot()
	kuka.publish_grip_cmd(gripper_reset)
	kuka.publish_grip_cmd(gripper_close)
	kuka.publish_grip_cmd(gripper_open)

	participant_name = input('input participant name: ')
	participant_height = input( 'input the height of the participant(cm): ' )	
	if participant_name == '':
		participant_name = 'test'
	if participant_height == '':
		participant_height = '168'

	for i in range(15):
		print('waiting for gripper init in {}s'.format(15-i))
		time.sleep(1)


	logger.info('Initializing gripper')
	return participant_name, participant_height



def manipulation_loop(kuka, distance, height, angle, yaw = 0):

	logger.info("Robot ready!!")
	#kuka.publish_pose(pre_grasping_pose)
	#logger.info("Position reached!!")
	########
	if sec_debug:
		pass
	else: 
		x,y,z= -0.55,0.0,0.4 # pre grasping position
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		logger.info("pre grasping position reached!!!")
		########
		x,y,z=-0.55,0.0,0.28
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		logger.info("Grasping position reached!!")
		########
		kuka.publish_grip_cmd(gripper_close)
		time.sleep(2)
		########
		x,y,z=-0.55, 0.0, 0.4
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		logger.info("Moving to human!!")
		time.sleep(0.5)
	########


	x,y,z= distance, -0.55, height
	# euler_angles = [ 3.14159, -1.5707,0 ]
	euler_angles = angle #[ 3.14159 , angle, yaw ]
	#euler_angles=kuka.get_angles([x,y,z],0.0)
	new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
	kuka.publish_pose(new_pose_msg)
	logger.info("Human operation position reached!!")
	time.sleep(1)


	if sec_debug:
		pass
	else:
		time.sleep(4)
		########
		x,y,z= -0.55,0.0,0.4
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		#logger.info("Putting object to target position!!")
		time.sleep(0.5)
		########
		x,y,z= -0.55,0.0,0.28
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		#logger.info("Human operation position reached!!")
		########
		kuka.publish_grip_cmd(gripper_open)
		########
		x,y,z= -0.55,0.0,0.4
		euler_angles=kuka.get_angles([x,y,z],0.0)
		new_pose_msg=kuka.get_message_pose([x,y,z],euler_angles,0.0)
		kuka.publish_pose(new_pose_msg)
		#logger.info("Human operation position reached!!")
		print('task finished!!')

	########
	# kuka.publish_grip_cmd(gripper_open)

	# kuka.home()
	########

def main_loop( chest_zero_position ):
	print(chest_zero_position)

	index = 0
	for distance in distances:
		for height in heights:
			for angle in angles:
				index += 1
				

				x = -0.2 + 0.24* math.sin( angle ) + distance
				y = -0.7
				z = chest_zero_position + height + 0.24* math.cos(angle)
				if debug:
					print('target_position:{}'.format([round(x,4),round(y,4),round(z,4), [PI, round(angle,4), 0 ]]))
					logger.info('experiment index: {:02d}\t distance: {} m, height: {} m, angle: {} '.format( index, 0.60 - distance, height, [ PI , angle, 0 ] ))
				else:
					logger.info('experiment index: {:02d}\t distance: {} m, height: {} m, angle: {} '.format( index, 0.60 - distance, height, [ PI , angle, 0 ] ))
					manipulation_loop(kuka, x, z, [ 3.14159 , angle, 0 ] )
		########
		kuka.publish_pose(waiting_pose)
		logger.info("pre grasping position reached!!!")

def second_loop( chest_zero_position ):
	print(chest_zero_position)
	angle = -PI/2
	index = 0
	for distance in distances:
		for height in heights:
			index += 1
			x = -0.2 + 0.24* math.sin( angle ) + distance
			y = -0.7
			z = chest_zero_position + height + 0.24* math.cos(angle)
			if debug:
				print('target_position:{}'.format([round(x,4),round(y,4),round(z,4), round(angle,4)]))
				logger.info('experiment index: {:02d}\t distance: {} m, height: {} m, angle: {} '.format( index, 0.60 - distance, height, [0,0, -PI] ))
			else:
				logger.info('experiment index: {:02d}\t distance: {} m, height: {} m, angle: {} '.format( index, 0.60 - distance, height, [0,0, -PI] ))
				# manipulation_loop(kuka, x, z, [PI, angle, 0] )
				manipulation_loop(kuka, x, z, [-1.570912730771581, -0.8795616756912527, -1.5706452313405894] )
		########
		kuka.publish_pose(waiting_pose)
		logger.info("pre grasping position reached!!!")



if __name__ == '__main__':
	debug = True
	sec_debug= True

	participant_name, participant_height = prepare(debug)
	
	logfile('./log/manipulation_recording_{}_{}.log'.format(participant_name, round(time.time())))
	logger.info('participant: {} '.format(participant_name))
	logger.info('participant height:{}'.format( participant_height ))
	estimated_table_chest_height = (5.55 / 7.5 * int(participant_height))/100 - TABLE_HEIGHT

	# main_loop(estimated_table_chest_height)
	# logger.info('Requires to change the grap position. press enter to continue when you changed the grab position')
	# _ = input('I confirmed I have everything done')
	second_loop(estimated_table_chest_height)





# [0.665288626384, -0.239578151821, 0.66529750824, -0.239525686603]




































