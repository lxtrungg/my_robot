#!/usr/bin/env python

import rospy
from my_robot_control.msg import  uwb_data
from geometry_msgs.msg import Point

import tf
import time
import math

import numpy as np
import math
from scipy.linalg import eigvals

all_distance = []
all_destination_id = []
pose_x = 0 
pose_y = 0
sensor_pos = []


rospy.init_node('localization_data_node', anonymous=True)
pub = rospy.Publisher('/my_robot/localization_data_topic', Point, queue_size=10)


def subscribe_data(uwb_data_cell):
    all_destination_id = uwb_data_cell.destination_id
    all_distance = uwb_data_cell.distance
    robot_pos = position_calculation(sensor_pos,all_distance)
    pos_meter = np.divide(robot_pos, 1000) 
    publish_data(pos_meter[0],pos_meter[1])    


def publish_data(pose_x,pose_y):
    robot_pos = Point()
    robot_pos.x = float(pose_x)
    robot_pos.y = float(pose_y)
    robot_pos.z = 0.0
    pub.publish(robot_pos)

    
#added => cagdas tekok 
def position_calculation(sensor_pos,dist): 
    A=(-2*sensor_pos).transpose()
    vertical=len(sensor_pos[0])
    horizontal=len(sensor_pos)
    B=np.ones([horizontal,1],dtype=float).transpose()
    A=np.append(A,B,axis=0)
    A=A.transpose()
    R=np.zeros([1,vertical],dtype=float).transpose() 
    b=np.zeros([1,horizontal],dtype=float)
    for i in range(horizontal):
        b[0][i]=dist[i]**2-(np.linalg.norm(sensor_pos[i]))**2    
    D=np.eye(3)
    D=np.append(D,np.zeros([3,1],dtype=float),axis=1)
    D=np.append(D,np.zeros([1,4],dtype=float),axis=0)
    f=np.zeros([4,1],dtype=float)
    np.put(f,3,-0.5)
    AtrA=np.matmul(A.transpose(),A)
    eival=eigvals(D,AtrA)
    eival=np.real(np.sort(eival))
    eival=eival[::-1]
    lam=-(eival**-1)
    ub=10**7
    lb=lam[0]
    tolerance = 10**-4
    count=0
    while ub-lb>tolerance:
        count+=1
        midpo = (ub + lb)/2
        yhat=np.linalg.solve((AtrA+midpo*D),np.subtract(np.matmul(A.transpose(),b.transpose()).transpose(),(midpo*f).transpose()).transpose())
        #fun = yhat'*D*yhat + 2*f'*yhat;
        fun=np.matmul(np.matmul(yhat.transpose(),D),yhat)+np.matmul(2*f.transpose(),yhat)
        if fun>0:
            lb = midpo
        else:
            ub = midpo
    x=yhat[:3]
    if yhat[3]<0:
        x=-x
    return x 
     

def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []   
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    while len(sensor_pos) != 8:
        for i in range(max_anchor):
            try:
                time.sleep(0.3)
                (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
                sensor_pos.append(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                break

    sensor_pos = np.dot(sensor_pos, 1000)
    rospy.loginfo("UWB Anchor List:\nWarning : uint is mm \n" + str(sensor_pos))

    return sensor_pos


if __name__ == "__main__":
    #get uwb anchors postion
    sensor_pos = get_anchors_pos()

    rospy.Subscriber('/my_robot/uwb_data_topic', uwb_data, subscribe_data)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()