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

engine = pyttsx3.init()
safe_mode=True

fps=5
SH=320
SW=480
PIXELS_TO_METERS=500
ROBOT={'L1':1.5*PIXELS_TO_METERS,'L2':1.5*PIXELS_TO_METERS}
AXIS_LENGHT=20
AXIS_WIDTH=2
RED=[0,0,255]
GREEN=[0,255,0]
BLUE=[255,0,0]
BLUE_GREEN=[255, 255, 0]
YELLOW=[255,0,255]
TRAINING_STEPS=10001

RS_CAM_X=-0.58
RS_CAM_Y=-0.24
RS_CAM_Z=1.21

u_default=0
v_default=150

ref_position=[0,0,0]

def make_intrinsics():
    intrinsics = rs.intrinsics()
    intrinsics.coeffs = [0,0,0,0,0]
    intrinsics.fx = 615.799 
    intrinsics.fy = 615.836
    intrinsics.height = 480
    intrinsics.ppx = 317.985
    intrinsics.ppy = 244.153
    intrinsics.width=640
    return intrinsics

def get_angles(pos,wrist):
    euler=[0,0,0]
    euler[0]=3.14159
    euler[2]=math.atan(float(pos[1])/float(pos[0]))-1.5707+wrist
    euler[2]=-1.5707+wrist
    #print(euler[2]*360/(2*3.1416))
    return euler
def get_message_pose(pos,euler,height):
    grasping_pose = PoseStamped()
    grasping_pose.pose.position.x = pos[0]
    grasping_pose.pose.position.y = pos[1]
    grasping_pose.pose.position.z = pos[2]+height
    q=quaternion_from_euler(euler[0],euler[1],euler[2])
    grasping_pose.pose.orientation.w = q[3]
    grasping_pose.pose.orientation.x = q[0]
    grasping_pose.pose.orientation.y = q[1]
    grasping_pose.pose.orientation.z = q[2]
    return grasping_pose

def open_gripper():
    kuka.publish_grip_cmd(gripper_pre_1)
    return True
def close_gripper():
    kuka.publish_grip_cmd(gripper_close)
    return True
def move_arm(move):
    euler=get_angles(move,0.0)
    grasping_pose=get_message_pose(move,euler,0.1)
    kuka.publish_pose(grasping_pose)
    return True

intr=make_intrinsics()

##Perception

class Environment():
    def __init__(self):
        self.objects={}
        self.goals={}
        self.hands={}
        self.data_objects=[]
        self.data_goals={}
        self.data_hands={}
        self.high_level_state="home"
        self.high_state_publisher = rospy.Publisher('/human_robot_experiment/high_level_state', String, queue_size=10)
        self.state_publisher_str = threading.Thread(target=self.publish_high_level_state)
        rospy.Subscriber('/irohms_idql/irohms_perception_table_elements', Elements, self.get_objects)
        rospy.Subscriber('/irohms_idql/irohms_perception_table_goals', Elements, self.get_goals)
        rospy.Subscriber('/irohms_idql/irohms_perception_table_hands', Elements, self.get_hands)
        self.state_publisher_str.start()
    def publish_high_level_state(self):
        r=rospy.Rate(10)
        while not rospy.is_shutdown():
            self.high_state_publisher.publish(self.high_level_state)
            r.sleep()
    def get_objects(self,data):
        self.data_objects=data
        #print(data)
    def observe_environment(self):
        self.objects={}
        self.goals={}
        try:
            for i in range(0,len(self.data_objects.key)):
                #print(self.data_objects.key[i])
                self.objects[self.data_objects.key[i]]={}
                self.objects[self.data_objects.key[i]]['position']=[self.data_objects.x[i],self.data_objects.y[i],0]
                self.objects[self.data_objects.key[i]]['orientation']=[0,0,0]
                self.objects[self.data_objects.key[i]]['lenght']=self.data_objects.width[i]
                self.objects[self.data_objects.key[i]]['width']=self.data_objects.lenght[i]
                self.objects[self.data_objects.key[i]]['height']=self.data_objects.height[i]
                self.objects[self.data_objects.key[i]]['shape']=self.data_objects.shape[i]    
                self.objects[self.data_objects.key[i]]['color']=[self.data_objects.r[i],self.data_objects.g[i],self.data_objects.b[i]]
                self.objects[self.data_objects.key[i]]['path']=[]
                self.objects[self.data_objects.key[i]]['grid']=[]
                self.objects[self.data_objects.key[i]]['path_smooth']=[]
                self.objects[self.data_objects.key[i]]['goal']=[]
        except:
            pass
     
        try:
            for i in range(0,len(self.data_goals.key)):
                #print(self.data_goals.x[i],self.data_goals.y[i])
                self.goals[self.data_goals.key[i]]={}
                self.goals[self.data_goals.key[i]]['position']=[self.data_goals.x[i],self.data_goals.y[i],0]
                self.goals[self.data_goals.key[i]]['orientation']=[0,0,0]
        except Exception as e:
            #print (e)
            pass
        try:
            for i in range(0,len(self.data_hands.key)):
                #print(self.data_goals.x[i],self.data_goals.y[i])
                self.hands[self.data_hands.key[i]]={}
                self.hands[self.data_hands.key[i]]['position']=[self.data_hands.x[i],self.data_hands.y[i],0]
                self.hands[self.data_hands.key[i]]['orientation']=[0,0,0]
                self.hands[self.data_hands.key[i]]['width']=30
                self.hands[self.data_hands.key[i]]['lenght']=20
                self.hands[self.data_hands.key[i]]['shape']='square'
                self.hands[self.data_hands.key[i]]['color']=[255,255,255]
                self.hands[self.data_hands.key[i]]['goal']=[]
                self.hands[self.data_hands.key[i]]['path']=[]
                self.hands[self.data_hands.key[i]]['grid']=[]
                self.hands[self.data_hands.key[i]]['path_smooth']=[]
                
        except Exception as e:
            #rospy.logerr(e)
            pass
        return self.objects, self.goals
    def get_goals(self,data):
        #rospy.loginfo(data.key)
        self.data_goals=data
        return
    def get_hands(self,data):
        #rospy.loginfo(data.key)
        
        self.data_hands=data
        #rospy.logerr(data)
        return



def running(obj,goal,m_field,robot):
    vertical,horizontal,state,x,y, iQtable,actions, x_offset, y_offset,iEnv =robot.set_env(obj,goal)
    #solve iQTable return set of actions 
    table=np.array(iQtable)
    actionsss=robot.agent(table,iEnv,x,y,state)
    cv2.startWindowThread()
    if actionsss is not None:
        for a in actionsss:
            robot.render(obj,vertical,horizontal,state, x_offset, y_offset,iQtable,m_field)
            robot.step(obj,a)
            robot.reset()
            key=cv2.waitKey(int(round(50/fps)))  # We need to call cv2.waitKey after cv2.imshow
            if key == 0:  # Press Esc for exit
                break
        #interpolation
        try:
            x =  [item[0] for item in robot.objects[obj]['path']]
            x.append(robot.objects[obj]['path'][-1][2])
            x.insert(0,robot.objects[obj]['path'][0][4])
            y =  [item[1] for item in robot.objects[obj]['path']]
            y.append(robot.objects[obj]['path'][-1][3])
            y.insert(0,robot.objects[obj]['path'][0][5])
            arr = np.array(list(zip(x,y)))
            x, y = zip(*arr)
            f, u = interpolate.splprep([x, y], s=2)
            xint, yint = interpolate.splev(np.linspace(0, 1, 20), f)
            robot.objects[obj]['path_smooth']=list(zip(xint,yint))
            robot.render(obj,vertical,horizontal,state, x_offset, y_offset,iQtable,True)
            return list(zip(xint,yint)),actionsss
        except Exception as e:
            print("Interpolation error")
            print(e)
            return None,actionsss
    else:
        print("Not possible to handle piece")
        return None,actionsss

def get_speech_keyword():  
    #return
    r = sr.Recognizer()
    try:
        with sr.Microphone() as source2: 
            r.adjust_for_ambient_noise(source2, duration=0.2)
            audio = r.listen(source2, timeout=3, phrase_time_limit=3)
            query = r.recognize_google(audio)
            print(query)
            word = query.lower()
            print(word)
            return word
            
    except Exception as e:
        rospy.logerr(e)
        rospy.loginfo("Not instruction spoken")
        return None

def get_speech():   
    #return
    r = sr.Recognizer()
    try:
        with sr.Microphone() as source2: 
            r.adjust_for_ambient_noise(source2, duration=0.2)
            audio = r.listen(source2, timeout=5, phrase_time_limit=12)
            query = r.recognize_google(audio)
            word = query.lower()
            return word
            
    except Exception as e:
        #rospy.logerr(e)
        rospy.loginfo("Not instruction spoken")
        return None

def speak_phrase(phrase):
    engine.say(phrase)
    engine.runAndWait()



def get_sub_goals(instructions,temp_objects):
    goals={}
    for ins in instructions:
        keys=[]
        for type_object in ins[0]:
            if type_object[0][2]=="*":
                for key in temp_objects.keys():
                    if type_object[0][:-1]== key[:2]:
                        keys.append(key)
            else:
                for key in temp_objects.keys():
                    if type_object[0]== key[:3]:
                        keys.append(key)
                pass
        if ins[1] is not None:
            goals[ins[1]]=keys
    return goals

###MATH utilities

def rad_to_deg(angle):
    pi=3.1416
    angle=(angle*360)/(2*pi)
    return round(angle,2)

###Robot control
def twod_to_threed(x,y,z):
    return rs.rs2_deproject_pixel_to_point(intr,[float(x),float(y)],z)
def get_robot_path(object_path,dist):
    path=[]
    path_torso=[]
    positions_realsense=twod_to_threed(object_path[0][0]+u_default,object_path[0][1]+v_default,dist)
    x=RS_CAM_X-positions_realsense[1]
    y=RS_CAM_Y-positions_realsense[0]
    z=RS_CAM_Z-positions_realsense[2]+0.2
    path.append([float(x),float(y),float(z)])
    #t1,t2,t3,t4,t5,t6,t7=inverse_kinematics_1(x,y,z)
    #path.append([float(t1),float(t2),float(t3),float(t4),float(t5),float(t6),float(t7)])
    #x=-0.535+object_path[0][0]*0.0025
    #y= 1.005-object_path[0][1]*0.0025
    print(x,y,z)
    z=RS_CAM_Z-1.17+0.14
    path.append([float(x),float(y),float(z)])
    #z=RS_CAM_Z-positions_realsense[2]+0.2
    #path.append([float(x),float(y),float(z)])
    #t1,t2,t3,t4,t5,t6,t7=inverse_kinematics_1(x,y,z)
    #path.append([float(t1),float(t2),float(t3),float(t4),float(t5),float(t6),float(t7)])
    path_torso.append([0.0,0.0])
    for point in object_path:
        positions_realsense=twod_to_threed(point[0]+u_default,point[1]+v_default,1.18)
        x=RS_CAM_X-positions_realsense[1]
        y=RS_CAM_Y-positions_realsense[0]
        z=RS_CAM_Z-positions_realsense[2]+0.25
        #print(x,y)
        #t1,t2,t3,t4,t5,t6,t7=inverse_kinematics_1(x,y,z)
        #print(rad_to_deg(t1))
        path.append([float(x),float(y),float(z)])
        path_torso.append([0.0,0.0])
    #TODO get kuka path
    return path,path_torso



def execute_step(path,path_torso,instructions,objeto,goal,objects_3,actions_set,x,y,sub_goal):
    #TODO init kuka
    time.sleep(0.5)
    move_arm(path[0])
    open_gripper()
    move_arm(path[1])
    close_gripper()
    for move in path[2:]:
        handle_arm = move_arm(move)
        #move kuka to position
        ###look for changes
        if safe_mode:
            _,goals_loc=env.observe_environment()
            goal=goals_loc[sub_goal]['position']
            hands=env.hands.copy()
            env.hands={}
            try:
                for hand,item in hands.items():
                    if len(item.keys())>0:
                        objects_3[hand]=item
            except Exception as e:
                rospy.logerr(e)
            try:
                objects_3[objeto]['position']=[x,y,0]
                objects_3[objeto]['path']=[]
                changes=RealTimeTracker(objects_3)
                error=changes.testing(objeto,goal,False,actions_set)
                print(error)
                if error:
                    speak_phrase("Becareful with your hand")
                    time.sleep(2)
                    while error:
                        
                        _,goals_loc=env.observe_environment()
                        goal=goals_loc[sub_goal]['position']
                        hands=env.hands.copy()
                        env.hands={}
                        try:
                            for hand,item in hands.items():
                                if len(item.keys())>0:
                                    objects_3[hand]=item
                        except Exception as e:
                            rospy.logerr(e)
                        try:
                            objects_3[objeto]['position']=[x,y,0]
                            objects_3[objeto]['path']=[]
                            changes=RealTimeTracker(objects_3)
                            error=changes.testing(objeto,goal,False,actions_set)
                        except Exception as e:
                            rospy.logerr(e)
                            continue
                    speak_phrase("Thank you")

                    
                #print(objects_3)
            except Exception as e:
                rospy.logerr(e)
                continue
        ##objects_3[objeto]['position']=[x,y,0]
        ##
        ##
        ##print(error)
        ####TODO if error  then plan from current position to goal 
    #handle_arm.wait()
    open_gripper()
    kuka.home()
    return True


if __name__ == '__main__':
    env=Environment()
    #kuka.init_robot()
    #time.sleep(10)
    rospy.loginfo("Robot ready!!")
    speak_phrase("Robot ready")
    speak_phrase("Waiting for instructions")
    while not rospy.is_shutdown():
        env.observe_environment() 
        print(env.objects.keys()) 
        if len(env.objects.keys())==0:
            rospy.loginfo("There are not objects to grasp on the table!")
        else: 
            elements=env.objects.keys()
            while not rospy.is_shutdown():
                indexes=0
                for element in elements:
                    answer=input('Move object?(y/n)')
                    if answer=="y" or answer=="Y":
                        env.high_level_state="start"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        #get element pose
                        #positions_realsense=twod_to_threed(env.objects[element]['position'][0]+u_default, env.objects[element]['position'][1]+v_default,env.objects[element]['height'])
                        #x=RS_CAM_X-positions_realsense[1]
                        #y=RS_CAM_Y-positions_realsense[0]
                        #z=RS_CAM_Z-positions_realsense[2]+0.2
                        ##reach above posicion
                        #move_arm([x,y,z])
                        #open_gripper()
                        env.high_level_state="above object"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        ##reach grasping position
                        
                        #z=RS_CAM_Z-1.17+0.14
                        #move_arm([x,y,z])
                        env.high_level_state="grasping position"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        #close gripper
                        
                        #close_gripper()
                        env.high_level_state="gripper closed"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)

                        ##reach above position
                        #z=RS_CAM_Z-positions_realsense[2]+0.2
                        #move_arm([x,y,z])
                        env.high_level_state="object lifted"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        #break
                        ###reach destination
                        env.high_level_state="in the way"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        #move_arm([-0.47+indexes,-0.5,0.18])
                        ###open gripper
                        env.high_level_state="gripper opened"
                        #open_gripper()
                        speak_phrase(env.high_level_state)
                        time.sleep(1)

                        ###reach destination
                        #move_arm([-0.47+indexes,-0.5,0.25])
                        env.high_level_state="above destination"
                        speak_phrase(env.high_level_state)
                        time.sleep(1)

                        ###home
                        env.high_level_state="finished"
                        #kuka.home()
                        speak_phrase(env.high_level_state)
                        time.sleep(1)
                        indexes+=0.08
                    elif  answer=="n" or answer=="N":
                        exit(1)
                    else:
                        rospy.logerr("Not a valid answer!!")
                
            
        
