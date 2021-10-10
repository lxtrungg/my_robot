#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
from math import atan2, sin, cos
pre_imu_raw = 0
imu_fake = 0

def subcriber_imu_callback(imu_data):
    global pre_imu_raw
    imu_raw = imu_data.vector.z
    delta = imu_raw - pre_imu_raw
    delta = atan2(sin(delta), cos(delta))
    pre_imu_raw = imu_raw
    publish_imu_fake(delta)
    
def publish_imu_fake(delta):
    global imu_fake
    imu_fake += delta
    data = Vector3()
    data.z = imu_fake
    print(imu_fake)
    imu_pub.publish(data)

def main():
    global imu_pub
    rospy.init_node('robot_imu_fake')
    imu_pub = rospy.Publisher('/my_robot/imu_fake', Vector3, queue_size=10)
    rospy.Subscriber('/my_robot/imu/rpy/filtered', Vector3Stamped, subcriber_imu_callback)
    rospy.spin()

if __name__ == '__main__':
    main()