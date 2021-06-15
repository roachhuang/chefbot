# Learning Robotics using Python 

#### [Learning Robotics using Python](http://learn-robotics.com) book tutorials source code
![book_cover](http://learn-robotics.com/images/section-image-1.jpg
 "Learning Robotics using Python")

### Buy book

* [PACKT](https://www.packtpub.com/application-development/learning-robotics-using-python)
* [Amazon.com](http://amzn.com/1783287535)
* [Amazon.in](http://www.amazon.in/dp/B00YEVZ6UK)

### Author

* [Lentin Joseph](https://in.linkedin.com/in/lentinjoseph)

### Installation
The code is comaptible with ROS Jade and ROS Indigo. The detail installation instruction of each packages is mentioned on the book

### Tutorials
* **Chapter 1**:  Introduction to Robotics
* **Chapter 2**: Mechanical design of a service Robot 
* **Chapter 3**: Working with Robot Simulation using ROS and Gazebo
* **Chapter 4**: Designing Chefbot Hardware 
* **Chapter 5**: Working with Robotic Actuators and Wheel Encoders 
* **Chapter 6**: Working with Robotic Sensors 
* **Chapter 7**: Programming Vision Sensors using Python and ROS 
* **Chapter 8**: Working with Speech Recognition and Synthesis using Python and ROS
* **Chapter 9**: Applying Artificial Intelligence to Chefbot using Python
* **Chapter 10**: Integration of Chefbot hardware and interfacing it into ROS, using Python
* **Chapter 11**: Designing a GUI for a robot using QT and Python 
* **Chapter 12**: The calibration and testing of Chefbot


# ROS Robotics Projects

#### [ROS Robotics Projects](http://rosrobots.com) 
![book_cover](http://rosrobots.com/img/ebook.png
 "ROS Robotics Projects")

### Buy book

* [PACKT](https://www.packtpub.com/hardware-and-creative/ros-robotics-projects)
* [Amazon.com](https://www.amazon.com/ROS-Robotic-Projects-Lentin-Joseph/dp/1783554711)
* [Amazon.in](https://www.amazon.in/ROS-Robotics-Projects-Lentin-Joseph-ebook/dp/B01MTJWNKI)


### Author

* [Lentin Joseph](https://in.linkedin.com/in/lentinjoseph)

### Installation
The code is comaptible with ROS melodic and ROS Indigo. The detail installation instruction of each packages is mentioned on the book

### Tutorials
* **Chapter 1:**  Getting Started with ROS Robotics Application Development
* **Chapter 2**:  Face Detection and Tracking Using ROS, OpenCV and Dynamixel Servos
* **Chapter 3**:  Building a Siri-Like Chatbot in ROS
* **Chapter 4**:  Controlling Embedded Boards Using ROS
* **Chapter 5**:  Teleoperate a Robot Using Hand Gestures
* **Chapter 6**:  Object Detection and Recognition
* **Chapter 7**:  Deep Learning Using ROS and TensorFlow
* **Chapter 8**:  ROS on MATLAB and Android
* **Chapter 9**:  Building an Autonomous Mobile Robot
* **Chapter 10**: Creating a Self-driving Car Using ROS!
* **Chapter 11**: Teleoperating Robot Using VR Headset and Leap Motion
* **Chapter 12**: Controlling Your Robots over the Web



# Mastering ROS for Robotics Programming 

#### [Mastering ROS for Robotics Programming](http://mastering-ros.com) book tutorials source code
![book_cover](http://mastering-ros.com/images/section-image-1.jpg
 "Mastering ROS for Robotics Programming")

### Buy book

* [PACKT](https://www.packtpub.com/hardware-and-creative/mastering-ros-robotics-programming)
* [Amazon.com](http://amzn.com/B0198DXFEW)
* [Amazon.in](http://www.amazon.in/dp/B0198DXFEW)


### Author

* [Lentin Joseph](https://in.linkedin.com/in/lentinjoseph)

### Installation
The code is comaptible with ROS Jade and ROS Indigo. The detail installation instruction of each packages is mentioned on the book

### Tutorials
* **Chapter 1:**  Introduction to ROS and its Package Management
* **Chapter 2**: Working with 3D Robot Modeling in ROS
* **Chapter 3**: Simulating Robots Using ROS and Gazebo
* **Chapter 4**: Using ROS MoveIt! and Navigation stack
* **Chapter 5**: Working with Pluginlib, Nodelets and Gazebo plugins
* **Chapter 6**: Writing ROS Controllers and Visualization plugins
* **Chapter 7**: Interfacing I/O boards, sensors and actuators to ROS
* **Chapter 8**: Programming Vision sensors using ROS, Open-CV and PCL
* **Chapter 9**: Building and interfacing a differential drive mobile robot hardware in ROS
* **Chapter 10**: Exploring advanced capabilities of ROS-MoveIt!
* **Chapter 11**: ROS for Industrial Robots
* **Chapter 12**: Troubleshooting and best practices in ROS

roslaunch turtlebot_teleop keyboard_teleop.launch


sudo apt-get install ros-melodic-depthimage-to-laserscan ros-melodic-
kobuki-gazebo-plugins ros-melodic-robot-pose-ekf ros-melodic-yocs-cmd-vel-
mux ros-melodic-move-base-msgs ros-melodic-openni-launch ros-melodic-
kobuki-description ros-melodic-gmapping ros-melodic-amcl ros-melodic-map-
server

https://wiki.ros.org/differential_drive/tutorials/setup

setpoint: /roachbot/lwheel/command
feedback: h/w interface read method
cmd: h/w interface write method

1. hub.launch
2. pc or pi.launch
3. heavy_load.launch

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

the ~ (tilde) at the beginning of the parameter name indicates that this is a private name specic to one node. Perhaps the easiest way to assign such a parameter is to include it between the node tags in a launch

1. unplug arduino and lidar cables
2. power on pi4
3. ssh pi4 (to make sure pi4 is on)
4. connect arduino cable and trun on the motor power
5. launch hub.launch and echo lwheel topic on pc to make sure hub working
6. launch pi.launch 
7. launch heavy_load from pc to test motors

8. // Navigation // 
9. kill heavy_load session
10. connect lidar cable to pi4 (make sure it starts to spin)
11. launch lidar from pi4
12. launch nav from pc


to do:
    connect imu's ad0 to ground so it address will be fixed at 0x68
    udev rules for fixing ttyusbX for a usb device
        0. Udev stores all the rules in the /etc/udev/rules.d/ 
        1. connect a device to a usb port, dmesg|grep 
        2. udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0) |grep serial
        3. look for serial and product
        4. ACTION=="add", ATTRS{product}=="<product>", ATTRS{serial}=="<serial>", SYMLINK+="arduion"
        5. uplug your devce and re-plug it in, and check the /dev/arduino
        =====================================================================
        rosrun turtlebot3_bringup create_udev_rules
        sudo udevadm control --reload
        sudo udevadm trigger
    refer to turtlebot setup to enable boot before wifi available
    study vel limitation of h/w interefece 
    tf2:
        sudo apt-get install ros-noetic-tf2-tools
    
    