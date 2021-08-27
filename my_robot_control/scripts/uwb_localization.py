#! /usr/bin/env python

import numpy as np
import rospy
import tf
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from uwb_filter.fusion_ekf import FusionEKF

class UWBLocalization(object):
    def __init__(self):
        self.kalman_filter = FusionEKF()

        self.anchor_poses = dict()

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        anchors_sub = rospy.Subscriber(anchors, MarkerArray, self.add_anchors)
        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, self.add_ranging)


        self.estimated_pose_pub = rospy.Publisher('/my_robot/uwb/pose', Odometry, queue_size=10)

        self.pose = Odometry()

        self.tf_listener = tf.TransformListener()

    def change_pose_ref(self, odom, tag, world='/map', base='/base_link'):
        """
        Retrieves the tf tag for the robot and then translates the odom position
        @param odom: The odometry that needs to be translated
        @param tag: The name of the tag to use i.e. /right_tag
        @param world: The base reference frame
        @param base: the transformed reference frame
        @return: the translated reference frame
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(world, tag, rospy.Time(0))

            pose = odom.pose.pose.position

            pose.x -= trans[0]
            pose.y -= trans[1]
            pose.z -= trans[2]

            (trans, rot) = self.tf_listener.lookupTransform(world, base, rospy.Time(0))

            pose.x += trans[0]
            pose.y += trans[1]
            pose.z += trans[2]

            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None
        """
        Function that handles the MarkerArray of anchor positions and updates the anchor pose dict
        @param msg: the MarkerArray topic message
        """
        for marker in msg.markers:
            self.anchor_poses[marker.id] = np.array(
                [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

    def add_ranging(self, msg):
        # type: (Ranging) -> None
        """
        Function that handles UWB range data and processes it through the Kalman filter
        @param msg: The Ranging topic message
        """

        if msg.anchorId in self.anchor_poses:
            anchor_pose = self.anchor_poses[msg.anchorId]
            anchor_distance = msg.range / 1000.
            self.kalman_filter.process_measurement(anchor_pose, anchor_distance)

    def run(self):
        """
        The step function that publishes the position information of the tags
        @return:
        """
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.pose.pose.pose.position.x = self.kalman_filter.kalman_filter.x[0]
            self.pose.pose.pose.position.y = self.kalman_filter.kalman_filter.x[1]
            self.pose.pose.pose.position.z = self.kalman_filter.kalman_filter.x[2]

            self.pose.twist.twist.linear.x = self.kalman_filter.kalman_filter.x[3]
            self.pose.twist.twist.linear.y = self.kalman_filter.kalman_filter.x[4]
            self.pose.twist.twist.linear.z = self.kalman_filter.kalman_filter.x[5]

            self.estimated_pose_pub.publish(self.pose)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("uwb_localization_kalman")
    loc = UWBLocalization()
    loc.run()
    rospy.spin()