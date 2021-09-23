#!/usr/bin/env python

import rospy, tf
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3, PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import PyKDL
from numpy import sin, cos, deg2rad

vel = {'v': 0.0 , 'w': 0.0}
allow_initialpose_pub = False
pos = rot = theta = 0
def publish_odometry(position, rotation):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position = position
    odom.pose.pose.orientation = rotation
    odom.twist.twist.linear  = Vector3(vel['v'], 0, 0)
    odom.twist.twist.angular = Vector3(0, 0, vel['w'])

    odom_pub.publish(odom)

def transform_odometry(position, rotation):
    
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'odom'
    trans.child_frame_id = 'base_footprint'
    trans.transform.translation = position
    trans.transform.rotation = rotation

    odom_broadcaster.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = position
    initialpose.pose.pose.orientation = rotation

    initialpose_pub.publish(initialpose)

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pos.x) > 0.001 or abs(y - pos.y) > 0.001 or abs(yaw - theta) > deg2rad(0.01):
        allow_initialpose_pub = True

if __name__ == '__main__':
    rospy.init_node('odom__pub')
    odom_pub = rospy.Publisher('/my_robot/odom', Odometry, queue_size=50)
    initialpose_pub = rospy.Publisher('/my_robot/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # rospy.Subscriber('/my_robot/amcl_pose', PoseWithCovarianceStamped, subcriber_amcl_callback)

    odom_broadcaster = tf.TransformBroadcaster()
    model = GetModelStateRequest()
    model.model_name = 'my_robot'
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        result = get_model_srv(model)
        pos = result.pose.position
        rot = result.pose.orientation
        linear = result.twist.linear
        theta = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
        vel['v'] = linear.x*cos(theta) + linear.y*sin(theta)
        vel['w'] = result.twist.angular.z
        publish_odometry(pos, rot)
        transform_odometry(pos, rot)
        rate.sleep()


