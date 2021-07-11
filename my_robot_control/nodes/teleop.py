#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import sys, select, termios, tty

LINEAR_STEP = 0.01
ANGULAR_STEP = 0.1

def getKey():
    try:
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('my_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    linear_speed = 0.3
    linear_vel = 0.0
    angular_speed = 1.82
    angular_vel  = 0.0
    try:
        while 1:
            key = getKey()
            if key =='w':
                linear_vel = linear_speed
                print('FORWARD   ', end='\r')
            elif key =='s':
                linear_vel = -linear_speed
                print('BACKWARD  ', end='\r')
            elif key =='d':
                angular_vel = -angular_speed
                print('RIGHTWARD ', end='\r')
            elif key =='a':
                angular_vel = angular_speed
                print('LEFTWARD  ', end='\r')
            elif key =='z':
                linear_speed += LINEAR_STEP
                print('INCREASE_SPEED',  end='\r')
            elif key =='x':
                linear_speed -= LINEAR_STEP
                print('DECREASE_SPEED',  end='\r')
            elif key == '':
                linear_vel = 0.0
                angular_vel = 0.0
                print('STOP     ',  end='\r')
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_vel

            pub.publish(twist)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

if __name__ == '__main__':
    main()
