#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print ('======================================')
    #print('range_min: %f', msg.range_min)
    #print('range_max: %f', msg.range_max)

    # unit: m
    print('front direction - s1 [0]')
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

rospy.init_node('laser_data')
sub = rospy.Subscriber('scan', LaserScan, callback)
rospy.spin()

    
