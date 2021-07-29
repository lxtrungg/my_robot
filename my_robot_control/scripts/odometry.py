#!/usr/bin/env python

import rospy, roslib, tf
import roslib
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

if __name__ == '__main__':
    rospy.init_node('odom_pub')
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    odom = Odometry()
    odom.child_frame_id = 'base_footprint'
    header = Header()
    header.frame_id = 'odom'
    br = tf.TransformBroadcaster()
    t = TransformStamped()
    model = GetModelStateRequest()
    model.model_name = 'my_robot'
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        result = get_model_srv(model)

        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        position = odom.pose.pose.position
        rotation = odom.pose.pose.orientation

        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation.x = rotation.x
        t.transform.rotation.y = rotation.y
        t.transform.rotation.z = rotation.z
        t.transform.rotation.w = rotation.w

        header.stamp = rospy.Time.now()
        odom.header = t.header = header
        br.sendTransformMessage(t)
        odom_pub.publish(odom)
        r.sleep()


