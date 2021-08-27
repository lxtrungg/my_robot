#!/usr/bin/env python
from re import X
import PyKDL
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf
from math import pi
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF
import sympy
from sympy import symbols, Matrix, sin, cos

pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
vel = {'v': 0.0, 'w': 0.0}
allow_initialpose_pub = False
imu_done = uwb_done = False
first_scan = False
ready_to_pub = False

class RobotEKF():
    def __init__(self, dim_x, dim_z, dim_u):

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = np.zeros((self.dim_x, 1))
        self.u = np.zeros((self.dim_u, 1))
        self.P = np.eye(self.dim_x)
        self.F = np.eye(self.dim_x)
        self.Q = np.eye(self.dim_x)
        self.R = np.eye(self.dim_z)
        self.y = np.zeros((self.dim_z, 1))
        self.S = np.zeros((self.dim_z, self.dim_z))
        self.K = np.zeros((self.x.shape))
        self.I = np.identity(self.dim_x)

        x, y, theta, v, w, t = symbols('x, y, theta, v, w, t')
        x_k = Matrix([x, y, theta])
        u_k = Matrix([v, w])

        self.fxu = Matrix([[x + v*t*cos(theta + 0.5*w*t)],
                           [y + v*t*sin(theta + 0.5*w*t)],
                           [theta + w*t]])
        
        self.hx = Matrix([[x], [y], [theta]])

        self.F = self.fxu.jacobian(Matrix(x_k))
        H = self.hx.jacobian(Matrix(x_k))
        self.H = np.array(H).astype(np.float)
        self.W = self.fxu.jacobian(Matrix(u_k))

        self.subs = {x: 0, y: 0, theta: 0, v: 0, w: 0, t: 0}
        self.x_x, self.x_y, self.x_theta = x, y, theta
        self.u_v, self.u_w = v, w
        self.time = t

    def predict_update(self, z, u, dt):
        self.subs[self.x_theta] = self.x[2, 0]
        self.subs[self.u_v] = u[0]
        self.subs[self.u_w] = u[1]
        self.subs[self.time] = dt

        F = np.array(self.F.evalf(subs=self.subs)).astype(float)
        W = np.array(self.W.evalf(subs=self.subs)).astype(float)
        P = self.P
        Q = self.Q
        R = self.R
        H = self.H
        I = self.I
        x = self.x

        #predict step
        x = F @ x
        P = F @ P @ F.T + W @ Q @ W.T

        #update step
        self.y = z - H @ x
        self.S = H @ P @ H.T + R
        SI = np.linalg.inv(self.S)
        self.K = P @ H.T @ SI
        self.x = x + self.K @ self.y

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        I_KH = I - self.K @ H 
        # self.P = I_KH @ P
        self.P = I_KH @ I_KH.T + self.K @ R @ self.K.T

        return self.x

def publish_odometry(position, rotation):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position = Point(*position)
    odom.pose.pose.orientation = Quaternion(*rotation)
    odom.twist.twist.linear  = Vector3(vel['v'], 0, 0)
    odom.twist.twist.angular = Vector3(0, 0, vel['w'])

    odom_pub.publish(odom)

def transform_odometry(position, rotation):
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'odom'
    trans.child_frame_id = 'base_footprint'
    trans.transform.translation = Vector3(*position)
    trans.transform.rotation = Quaternion(*rotation)

    odom_broadcaster.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = Point(*position)
    initialpose.pose.pose.orientation = Quaternion(*rotation)

    initialpose_pub.publish(initialpose)

def subscriber_uwb_callback(uwb_data):
    global previous_time, first_scan, ready_to_pub, pose, uwb_done
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    if not (first_scan and imu_done):
        first_scan = True
        ready_to_pub = True
        return

    z = np.array([[uwb_data.x, uwb_data.y, imu_data_yaw]]).T
    u = np.array([vel['v'], vel['w']])
    x_posterior = ekf.predict_update(z, u, dt)
    pose['x']   = x_posterior[0,0]
    pose['y']   = x_posterior[1,0]
    pose['yaw'] = x_posterior[2,0]

    uwb_done = True

def subscriber_odom_callback(odom_data):
    global vel
    vel['v'] = odom_data.twist.twist.linear.x
    vel['w'] = odom_data.twist.twist.angular.z

def subscriber_imu_callback(imu_data):
    global imu_data_yaw, imu_done
    imu_data_yaw = imu_data.vector.z
    imu_done = True
    # print('imu:' + str(imu_data_yaw))

def subscriber_imu_raw_callback(imu_data):
    global imu_data_yaw, imu_done
    # rot = imu_data.orientation
    # imu_data_yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    # imu_done = True
    # print('raw:' + str(imu_data_yaw))

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pose['x']) > 0.01 or abs(y - pose['y']) > 0.01 or abs(yaw - pose['yaw']) > 0.005:
        allow_initialpose_pub = True

def main():
    rospy.init_node('robot_ekf_pub', anonymous=False)
    global odom_pub, odom_broadcaster, initialpose_pub, allow_initialpose_pub, vel
    odom_pub = rospy.Publisher("/my_robot/odom", Odometry, queue_size=10)
    initialpose_pub = rospy.Publisher("/my_robot/initialpose", PoseWithCovarianceStamped, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    # rospy.wait_for_service('/gazebo/get_model_state')
    # get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rospy.Subscriber('/my_robot/odom_gazebo', Odometry, subscriber_odom_callback)
    rospy.Subscriber('/my_robot/imu/rpy/filtered', Vector3Stamped, subscriber_imu_callback)
    rospy.Subscriber('/my_robot/imu/data_raw', Imu, subscriber_imu_raw_callback)
    rospy.Subscriber('/my_robot/localization_data_topic', Point, subscriber_uwb_callback)
    rospy.Subscriber('/my_robot/amcl_pose', PoseWithCovarianceStamped, subcriber_amcl_callback)

    global previous_time
    previous_time = rospy.Time.now()

    # model = GetModelStateRequest()
    # model.model_name = 'my_robot'

    rospy.loginfo('Start node robot_ekf_pub')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if ready_to_pub:
            # result = get_model_srv(model)
            # pos = result.pose.position
            # rot = result.pose.orientation
            # vel['v'] = result.twist.linear.x
            # vel['w'] = result.twist.angular.z
            position = (pose['x'], pose['y'], 0)
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            print(pose)
            # print('first:' + str(position) + ' ' + str(pose['yaw']))
            # position = (result.pose.position.x, result.pose.position.y, 0)
            # rotation = (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w)
            # print('second:' + str(position) + ' ' + str(PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]))
            if imu_done and uwb_done:
                publish_odometry(position, rotation)
                transform_odometry(position, rotation)

                if allow_initialpose_pub:
                    allow_initialpose_pub = False
                    publisher_initialpose(position, rotation)
        rate.sleep()
    
if __name__ =='__main__':
    std_v = 0.05
    std_w = 0.05
    ekf = RobotEKF(dim_x=3, dim_z=3, dim_u=2)
    ekf.x = np.array([0, 0, 0]).reshape(3,1)
    ekf.P = np.eye(3)*1
    ekf.Q = np.diag([std_v, std_w])**2
    ekf.R = np.diag([.2, .2, .1])**2
    main()