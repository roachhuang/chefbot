#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs import Twist

def callback(msg):
    print ('======================================')
    #print('range_min: %f', msg.range_min)
    #print('range_max: %f', msg.range_max)

    # unit: m
    print('front direction - s1 [0]')
    # the lidar returns a vect of 359 val, being the initial val the corresponding to the robot.
    print msg.ranges[0]

    print('left direction - s2 [90]')
    print msg.ranges[90]

    print('rear direction - s3 [180]')
    print msg.ranges[180]

    # -90 degrees
    print('right direction - s4 [270]')
    print msg.ranges[270]

    print('s5 [359]')
    print msg.ranges[359]

    # if the distance to an obstacle in front of the robot is bigger than 0.5m, it'll move forward
    if msg.range[0] > 0.5:
        move.linear.x = 0.3
        move.angular.z = 0.0
    else:
        move linear.x = 0.0
        mov.angular.z = 0.0
    pub.publish(move)

rospy.init_node('laser_data')
sub = rospy.Subscriber('scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()
rospy.spin()

    
