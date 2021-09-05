#!/usr/bin/env python

import rospy
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, atan2, pi

x_DR = np.zeros((3, 1))
x_UWB = np.zeros((3, 1))
x_AMCL = np.zeros((3, 1))
x_EKF = np.zeros((3, 1))
th_IMU = []
move_done = False

vel = {'v': 0.0 , 'w': 0.0}
def subscriber_uwb_callback(uwb_data):
    global x_UWB
    # if abs(uwb_data.x) > 3.8 or abs(uwb_data.y) > 3.8:
    #     return
    pose = np.array([[uwb_data.x], [uwb_data.y], [uwb_data.z]])
    x_UWB = np.hstack((x_UWB, pose))

def subscriber_amcl_callback(amcl_data):
    global x_AMCL
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    # if abs(x) > 3.8 or abs(y) > 3.8:
    #     return
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_AMCL = np.hstack((x_AMCL, pose))

def subscriber_odom_callback(odom_data):
    global x_EKF
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    # if abs(x) > 3.8 or abs(y) > 3.8:
    #     return
    rot = odom_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_EKF = np.hstack((x_EKF, pose))

def subscriber_imu_callback(imu_data):
    global th_IMU
    th_IMU = np.hstack((th_IMU, imu_data.vector.z))

def subscriber_move_callback(done_data):
    global move_done
    move_done = done_data.data

def dead_reckoning(pose):
    global previous_time, x_DR
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    pose['x']   += vel['v']*dt*cos(pose['yaw'] + 0.5*vel['w']*dt)
    pose['y']   += vel['v']*dt*sin(pose['yaw'] + 0.5*vel['w']*dt)
    pose['yaw']  += vel['w']*dt
    pose['yaw'] = atan2(sin(pose['yaw']), cos(pose['yaw']))
    arr = np.array([[pose['x'], pose['y'], pose['yaw']]]).T
    x_DR = np.hstack((x_DR, arr))

    return pose

def plot():
    plt.figure(figsize=(18, 8))
    plt.subplot(121)
    plt.title('X-Y Graph')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid('-')
    linewidth = 1.3
    # x = [3.5, 3.5, -3.5, -3.5, 3.5]
    # y = [3.5, -3.5, -3.5, 3.5, 3.5]

    # plt.plot(x, y, 'o--k', linewidth = 1)

    plt.plot(x_DR[0, 1:].flatten(), x_DR[1, 1:].flatten(), '-.y', linewidth = linewidth, label = 'xy_DR')

    plt.plot(x_UWB[0, 1:].flatten(), x_UWB[1, 1:].flatten(), '--g', linewidth = linewidth, label = 'xy_UWB')

    plt.plot(x_AMCL[0, 1:].flatten(), x_AMCL[1, 1:].flatten(), ':c', linewidth = linewidth, label = 'xy_AMCL')

    plt.plot(x_EKF[0, 1:].flatten(), x_EKF[1, 1:].flatten(), '-r', linewidth = linewidth, label = 'xy_EKF')

    plt.legend(loc='upper right')
    
    plt.subplot(122)
    plt.title('YAW Graph')
    plt.xlabel('Sample (n)')
    plt.ylabel('Yaw (degree)')
    plt.ylim(-200, 200)
    plt.grid(True)
    
    plt.plot(np.rad2deg(x_DR[2, 1:].flatten()), '-y', linewidth = linewidth, label='yaw_DR')
    plt.plot(np.rad2deg(th_IMU), '-g', linewidth = linewidth, label='yaw_IMU')
    plt.plot(np.rad2deg(x_EKF[2, 1:].flatten()), '-r', linewidth = linewidth, label='yaw_EKF')
    plt.legend(loc='upper right')
    plt.show()

def main():
    rospy.init_node('robot_debug', anonymous=True)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Subscriber('/my_robot/localization_data_topic', Point, subscriber_uwb_callback)
    rospy.Subscriber('/my_robot/amcl_pose', PoseWithCovarianceStamped, subscriber_amcl_callback)
    rospy.Subscriber('/my_robot/odom', Odometry, subscriber_odom_callback)
    rospy.Subscriber('/my_robot/imu/rpy/filtered', Vector3Stamped, subscriber_imu_callback)
    rospy.Subscriber('/my_robot/move_done', Bool, subscriber_move_callback)
    model = GetModelStateRequest()
    model.model_name = 'my_robot'

    rospy.loginfo('Start node robot_debug')
    rate = rospy.Rate(20)
    cnt = 0
    global x_DR, x_UWB, x_AMCL, x_EKF
    global move_done
    global vel, previous_time
    previous_time = rospy.Time.now()
    first_scan = False
    while not rospy.is_shutdown():
        result = get_model_srv(model)
        pos = result.pose.position
        rot = result.pose.orientation
        linear = result.twist.linear
        theta = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
        vel['v'] = linear.x*cos(theta) + linear.y*sin(theta)
        vel['w'] = result.twist.angular.z
        if not first_scan:
            first_scan = True
            poseDR = {'x': pos.x, 'y': pos.y, 'yaw': theta}
        dead_reckoning(poseDR)

        if move_done:
            move_done = False
            rospy.loginfo('Show plot')
            plot()

        rate.sleep()

if __name__ =='__main__':
    main()