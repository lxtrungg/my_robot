#!/usr/bin/env python

import rospy

from my_robot_control.msg import uwb_data
from gazebo_msgs.msg import ModelStates
import tf 

import math
import numpy as np

import time
import threading
import os, sys
import random


global robot_pose_x,robot_pose_y,robot_pose_z
robot_pose_x =0
robot_pose_y =0
robot_pose_z =0

global counter
counter = 0 

rospy.init_node('uwb_simulation', anonymous=True)
#distances are publishing with uwb_data_topic
pub = rospy.Publisher('/my_robot/uwb_data_topic', uwb_data, queue_size=10)

def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []   
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    index = 4 if MODELSTATE_INDEX == 6 else 8
    while len(sensor_pos) != index:
        for i in range(max_anchor):
            try:
                time.sleep(0.3)
                (trans,rot) = listener.lookupTransform('map', uwb_id+str(i), rospy.Time(0))
                sensor_pos.append(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                break

    sensor_pos = np.dot(sensor_pos, 1000)
    rospy.loginfo("UWB Anchor List:\nWarning : uint is mm \n" + str(sensor_pos))
    return sensor_pos

def calculate_distance(uwb_pose):
    #pose comes in gazebo/model_states (real position)
    global robot_pose_x,robot_pose_y,robot_pose_z
    robot_pose = [robot_pose_x, robot_pose_y, robot_pose_z]

    #describe 2 points
    p1 = np.array(uwb_pose)
    p2 = np.array(robot_pose)

    #difference between robot and uwb distance
    uwb_dist = np.sum((p1-p2)**2, axis=0)
    #add noise 
    # uwb_dist=uwb_dist+np.random.normal(0, uwb_dist*0.01, 1)  
    return np.sqrt(uwb_dist)


def uwb_simulate(sensor_pos):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        all_distance = [] 
        all_destination_id = []

        for i in range(len(sensor_pos)):
            #calculate distance uwb to robot for all anchors 
            dist = calculate_distance(sensor_pos[i])   
            all_distance.append(dist) 
        
        #uwb_anchors_set.launch same order (not important for simulation)
        all_destination_id.append(0x6900)
        all_destination_id.append(0x6910)
        all_destination_id.append(0x6920)
        all_destination_id.append(0x6930)
        if MODELSTATE_INDEX != 6:
            all_destination_id.append(0x6940)
            all_destination_id.append(0x6950)
            all_destination_id.append(0x6960)
            all_destination_id.append(0x6970)
            
        #publish data with ROS             
        publish_data(all_destination_id , all_distance)  

        rate.sleep()  


def publish_data(all_destination_id, all_distance):
    #uwb message type is a special message so that firstly describe this message 
    uwb_data_cell = uwb_data()
    uwb_data_cell.destination_id=all_destination_id
    uwb_data_cell.stamp = [rospy.Time.now(),rospy.Time.now(),rospy.Time.now()]
    uwb_data_cell.distance = all_distance
    pub.publish(uwb_data_cell)


def subscribe_data(ModelStates):
    #for the get real position of robot subscribe model states topic  
    global robot_pose_x,robot_pose_y,robot_pose_z
    global counter
    counter = counter +1 

    #gazebo/modelstate topic frequency is 100 hz. We descrese 10 hz with log method 
    if counter%100 ==  0:  
        counter = 0 

        #ModelStates.pose[2] = turtlebot3 model real position on modelstates   
        robot_pose_x =ModelStates.pose[MODELSTATE_INDEX].position.x*1000
        robot_pose_y =ModelStates.pose[MODELSTATE_INDEX].position.y*1000
        robot_pose_z =ModelStates.pose[MODELSTATE_INDEX].position.z*1000
        

if __name__ == "__main__":
    #get uwb anchors postion
    
    MODELSTATE_INDEX = rospy.get_param('~modelstate_index', 6)
    sensor_pos = []
    sensor_pos = get_anchors_pos()

    rospy.loginfo("%s is %s", rospy.resolve_name('~modelstate_index'), MODELSTATE_INDEX)


    time.sleep(0.5)

    #get robot real position => you can change ModelStates.pose[] different robot's
    rospy.Subscriber('gazebo/model_states', ModelStates, subscribe_data)

    #start the publish uwb data
    uwb_simulate(sensor_pos)
    rospy.spin()
    
sys.exit()