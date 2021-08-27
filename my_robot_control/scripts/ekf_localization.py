#!/usr/bin/env python
import rospy 

import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from my_robot_control.msg import uwb_data

import tf 
import time


##initialize belief
global mu 
mu = [0,0, 0.0]
global sigma 
sigma = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
sensor_pos=[]

rospy.init_node('kalman_filter_localization', anonymous=True)
pub_ = rospy.Publisher('/my_robot/localization_data_topic', Point, queue_size=10)

MODELSTATE_INDEX = rospy.get_param('~modelstate_index', 6)

r = rospy.Rate(50)


def prediction_step(Odometry):
    global mu
    global sigma
    # Updates the belief, i.e., mu and sigma, according to the motion 
    # model
    # 
    # mu: 3x1 vector representing the mean (x,y,theta) of the 
    #     belief distribution
    # sigma: 3x3 covariance matrix of belief distribution 
    
    x = mu[0]
    y = mu[1]
    theta = mu[2]

    delta_vel = Odometry.twist.twist.linear.x * 1000           # redefine r1                  odom=>twist=>linear=>x 
    delta_w = Odometry.twist.twist.angular.z                  # redefine t                   odom=>twist=>angular=>z
    
    noise = 0.1**2
    v_noise = delta_vel**2
    w_noise = delta_w**2

    sigma_u = np.array([[noise + v_noise, 0.0],[0.0, noise + w_noise]])
    
    B = np.array([[np.cos(theta), 0.0],[np.sin(theta), 0.0],[0.0, 1.0]])


    #noise free motion
    x_new = x + delta_vel*np.cos(theta)/30
    y_new = y + delta_vel*np.sin(theta)/30
    theta_new = theta + delta_w/30
    
    #Jakobian of g with respect to the state
    G = np.array([[1.0, 0.0, -delta_vel * np.sin(theta)],
                    [0.0, 1.0, delta_vel * np.cos(theta)],
                    [0.0, 0.0, 1.0]])
    

    
    #new mu and sigma
    mu = [x_new, y_new, theta_new]
    sigma = np.dot(np.dot(G, sigma), np.transpose(G)) + np.dot(np.dot(B, sigma_u), np.transpose(B))
    #publish data every odom step 
    # mu_meter = np.divide(mu, 1000)
    # publish_data(mu_meter[0],mu_meter[1])
    
    return mu,sigma


def correction_step(uwb_data,  sensor_pos):
    global mu
    global sigma
    # updates the belief, i.e., mu and sigma, according to the
    # sensor model
    # 
    # The employed sensor model is range-only
    #
    # mu: 3x1 vector representing the mean (x,y,theta) of the 
    #     belief distribution
    # sigma: 3x3 covariance matrix of belief distribution 

    x = mu[0]
    y = mu[1]
    z = 0 
    theta = mu[2]

    #measured landmark ids and ranges
    ids = uwb_data.destination_id
    ranges =uwb_data.distance
    # Compute the expected range measurements for each landmark.
    # This corresponds to the function h
    H = []
    Z = []
    expected_ranges = []
    for i in range(len(ids)):
        lm_id = ids[i]
        meas_range = ranges[i]
        lx = sensor_pos[i][0]
        ly = sensor_pos[i][1]
        lz = sensor_pos[i][2]
        #calculate expected range measurement
        range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2+(lz - z)**2 )
        #compute a row of H for each measurement
        H_i = [(x - lx)/range_exp, (y - ly)/range_exp, 0]
        H.append(H_i)
        Z.append(ranges[i])
        expected_ranges.append(range_exp)
    # noise covariance for the measurements
    R = 0.1 * np.eye(len(ids))
    # Kalman gain
    K_help = np.linalg.inv(np.dot(np.dot(H, sigma), np.transpose(H)) + R)
    K = np.dot(np.dot(sigma, np.transpose(H)), K_help)
    # Kalman correction of mean and covariance
    mu = mu + np.dot(K, (np.array(Z) - np.array(expected_ranges)))
    sigma = np.dot(np.eye(len(sigma)) - np.dot(K, H), sigma)
    mu_meter = np.divide(mu, 1000)
    publish_data(mu_meter[0],mu_meter[1])

    return mu,sigma


def subscribe_odom_data(Odometry):
    [mu,sigma]=prediction_step(Odometry)
    

def subscribe_uwb_data(uwb_data):
    [mu,sigma]=correction_step(uwb_data, sensor_pos)


def publish_data(pose_x,pose_y):
    robot_pos_ = Point()
    robot_pos_.x = float(pose_x)
    robot_pos_.y = float(pose_y)
    robot_pos_.z = 0

    pub_.publish(robot_pos_)

def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    index = 0
    index = 4 if MODELSTATE_INDEX == 6 else 8
    while len(sensor_pos) != index:
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
    
    rospy.Subscriber("/my_robot/odom_gazebo", Odometry, subscribe_odom_data)
    rospy.Subscriber("/my_robot/uwb_data_topic", uwb_data, subscribe_uwb_data)

    rospy.spin()