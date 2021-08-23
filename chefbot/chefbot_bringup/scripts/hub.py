#!/usr/bin/env python

"""
launchpad_node.py - Receive sensor values from Launchpad board and publish as topics
Created September 2014
Copyright(c) 2014 Lentin Joseph
Some portion borrowed from  Rainer Hessmer blog
http://www.hessmer.org/blog/
"""

import threading
# import math
# import sys
import serial
from time import sleep

# Python client library for ROS
import rospy
# This module helps to receive values from serial port
# from SerialDataGateway import SerialDataGateway
# Importing ROS data type for IMU
from sensor_msgs.msg import Imu
# Importing ROS data types
from std_msgs.msg import Int64, Float32, String, Header

# Class to handle serial data from Launchpad and converted to ROS topics

class LaunchpadClass():
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        # Sensor variables
        self._Counter = 0
        self._left_encoder_value = 0
        self._right_encoder_value = 0
        self._lwheel_vel_value = 0
        self._rwheel_vel_value = 0
        
        self._left_wheel_speed_ = 0
        self._right_wheel_speed_ = 0

        self._LastUpdate_Microsec = 0
        self._Second_Since_Last_Update = 0
        # self.robot_heading = 0

        # Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
        # self._SerialDataGateway = SerialDataGateway(port, baudRate, self._HandleReceivedLine)
        # rospy.loginfo("Started serial communication")
        self.ser = serial.Serial(port, baudrate, timeout=None, write_timeout=1)
        # self.ser.open()
        self.ser.flush()
        # wait for the arduino to reset
        sleep(2)
        # call every 1s
        #rospy.Timer(rospy.Duration(1), self.timerCB)

        # Subscribers and Publishers
        # Publisher for left and right wheel encoder values
        self._Left_Encoder = rospy.Publisher('lwheel', Int64, queue_size=10)
        self._Right_Encoder = rospy.Publisher('rwheel', Int64, queue_size=10)
        self.pub_lvel = rospy.Publisher('lwheel_vel', Float32, queue_size=10)
        self.pub_rvel = rospy.Publisher('rwheel_vel', Float32, queue_size=10)     

        # Publisher for entire serial data
        self._SerialPublisher = rospy.Publisher(
            'serial', String, queue_size=30)
       
        # self.cal_offset = 0.0
        # self.orientation = 0.0
        # self.cal_buffer = []
        # self.cal_buffer_length = 1000

        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)

        """ what for? - figure it out
        self.deltat = 0
        self.lastUpdate = 0
        # New addon for computing quaternion
        self.pi = 3.14159
        self.GyroMeasError = float(self.pi * (40 / 180))
        self.beta = float(math.sqrt(3 / 4) * self.GyroMeasError)

        self.GyroMeasDrift = float(self.pi * (2 / 180))
        self.zeta = float(math.sqrt(3 / 4) * self.GyroMeasDrift)

        self.beta = math.sqrt(3 / 4) * self.GyroMeasError
        self.q = [1, 0, 0, 0]
        """

        # Speed subscriber
        # combine 2 motors speed into one topic!
        self._left_motor_speed = rospy.Subscriber(
            'left_wheel_speed', Float32, self._Update_Left_Speed)
        self._right_motor_speed = rospy.Subscriber(
            'right_wheel_speed', Float32, self._Update_Right_Speed)

    def _Update_Left_Speed(self, left_speed):
        self._left_wheel_speed_ = left_speed.data
        # rospy.loginfo(left_speed.data)
        # speed_message = 's %f %f\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
        speed_message = 's %.2f %.2f\r' % (self._left_wheel_speed_, self._right_wheel_speed_)
        self._WriteSerial(speed_message)

    def _Update_Right_Speed(self, right_speed):
        self._right_wheel_speed_ = right_speed.data
        # rospy.loginfo(right_speed.data)
        speed_message = 's %.2f %.2f\r' % (self._left_wheel_speed_, self._right_wheel_speed_)
        self._WriteSerial(speed_message)

    # Calculate orientation from accelerometer and gyrometer
    def _HandleReceivedLine(self, line):
        # self._Counter = self._Counter + 1
        # self._SerialPublisher.publish(String(str(self._Counter) + ", in: " + line))

        # if (len(line) > 0):
        lineParts = line.split('\t')
        try:
            if (lineParts[0] == 'e'):
                self._left_encoder_value = long(lineParts[1])
                self._right_encoder_value = long(lineParts[2])
                self._Left_Encoder.publish(self._left_encoder_value)
                self._Right_Encoder.publish(self._right_encoder_value)

            elif (lineParts[0] == 'v'):
                self._rwheel_vel_value = float(lineParts[1])
                self._lwheel_vel_value = float(lineParts[2])
                self.pub_lvel.publish(self._lwheel_vel_value)
                self.pub_rvel.publish(self._rwheel_vel_value)

                #self.ser.reset_input_buffer()                               

            elif (lineParts[0] == 'i'):
                self._qx = float(lineParts[1])
                self._qy = float(lineParts[2])
                self._qz = float(lineParts[3])
                self._qw = float(lineParts[4])
                    
                imu_msg = Imu()
                h = Header()
                h.stamp = rospy.Time.now()
                h.frame_id = 'base_link'
                imu_msg.header = h

                imu_msg.orientation_covariance = (-1., )*9
                imu_msg.angular_velocity_covariance = (-1., )*9
                imu_msg.linear_acceleration_covariance = (-1., )*9

                imu_msg.orientation.x = self._qx
                imu_msg.orientation.y = self._qy
                imu_msg.orientation.z = self._qz
                imu_msg.orientation.w = self._qw

                self.imu_pub.publish(imu_msg)
            else:
                rospy.logerr('wrong data: %s', lineParts[0])
        except:
            rospy.logwarn("Error in Sensor values")
            rospy.logwarn(lineParts)
            pass

    def _WriteSerial(self, message):
        # self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        # self._SerialDataGateway.Write(message)
        try:
            self.ser.write(message)
        except serial.SerialTimeoutException as e:
            rospy.logerr('data write: %b', self.ser.is_open)
            self.ser.open()
            raise e

    def ReadFromPort(self):
        r=rospy.Rate(100)
        while 1:
            #if self.ser.is_open == False:
            #    rospy.logerr('disconnect')
            if self.ser.in_waiting > 0:
                try:
                    line=self.ser.readline()
                    self._HandleReceivedLine(line)
                except:
                    rospy.logerr('cannot read!')
                    self.ser.open()
                    pass
            r.sleep()

    def Start(self):
        rospy.logdebug("Start receiving...")
        thread = threading.Thread(target=self.ReadFromPort)
        thread.start()               

        # self._SerialDataGateway.Start()

    def Stop(self):
        rospy.logdebug("Stopping")        
        self.ser.close()
        #self._SerialDataGateway.Stop()

    # def Subscribe_Speed(self):
    #    a = 1

    def Reset_Launchpad(self):
        # print "Reset"
        reset='r\r'
        self._WriteSerial(reset)
        sleep(1)
        self._WriteSerial(reset)
        sleep(2)

    def timerCb(self, event):
        # this is to prevent readline stop reading after awhile
        self.ser.reset_input_buffer()

    #    a = 3


if __name__ == '__main__':
    rospy.init_node('hub', anonymous=True)

    # Get serial port and baud rate of Tiva C Launchpad
    # ~ means local params
    port=rospy.get_param("~port", "/dev/ttyUSB0")
    baudRate=int(rospy.get_param("~baudRate", 115200))
    rospy.loginfo("Starting with serial port: " +
                  port + ", baud rate: " + str(baudRate))
    launchpad=LaunchpadClass(port, baudRate)
    try:       
        launchpad.Start()
        # a spin causes the main program to suspend, but keeps the callback function alive.
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Error in main function")
        launchpad.Reset_Launchpad()
        launchpad.Stop()
    except serial.SerialException as e:
        raise e
