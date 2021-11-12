The realtime_tools::RealtimePublisher allows users that write C++ realtime controllers to publish messages on a ROS topic from a hard realtime loop. The normal ROS publisher is not realtime safe, and should not be used from within the update loop of a realtime controller. The realtime publisher is a wrapper around the ROS publisher; the wrapper creates an extra non-realtime thread that publishes messages on a ROS topic.
////////////////////////////////////////////////////////////////////////////////////////////////////////

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

0. passwd - Sxx1x5
1. unplug arduino and lidar cables
2. power on pi4
3. ssh pi4 (to make sure pi4 is on)
4. connect arduino cable and trun on the motor power
5. launch hub.launch and echo lwheel topic on pc to make sure hub working
6. launch pi.launch 
7. launch heavy_load from pc to test motors

// Navigation // 
1. roslaunch chefbot_bringup hub.launch from pi
2. connect lidar cable to pi4 (make sure it starts to spin)
3. roslaunch slam slam.launch from pi   // run pi(run camera) and lidar
4. roslaunch nav nav.launch from pc
5. refer to a systematic approach page 454 
6. rosrun rqt_reconfigure rqt_reconfigure
7. tunning: l_inflation_radius: < 0.09, g_inflation_radius: 0.11 ~ 0.17, cost_scaling_factor:3.0

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

python3 -m pip install pyserial
sudo adduser your_username dialout
install urdf-tutorials

to do:
    sudo systemctl start roscore
    sudo systemctl daemon-reload
    systemctl status roscore
    sudo systemctl enable roscore

    auto ros on startup:
        https://qiita.com/strv/items/535a370842ae60af9b64
        https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/
        /etc/systemd/system/roscore.service
            [Unit]
            After=NetworkManager.service time-sync.target
            [Service]
            Type=forking
            User=[TODO enter user name here]
            # Start roscore as a fork and then wait for the tcp port to be opened
            # —————————————————————-
            # Source all the environment variables, start roscore in a fork
            # Since the service type is forking, systemd doesn’t mark it as
            # ‘started’ until the original process exits, so we have the
            # non-forked shell wait until it can connect to the tcp opened by
            # roscore, and then exit, preventing conflicts with dependant services
            ExecStart=/bin/sh -c “./opt/ros/melodic/setup.sh; . /etc/ros/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done”
            [Install]
            WantedBy=multi-user.target
        /etc/ros/env.sh
            #!/bin/sh
            export ROS_HOSTNAME=$(hostname).localexport ROS_MASTER_URI=http://$ROS_HOSTNAME:11311﻿
        /etc/systemd/system/roslaunch.service
            [Unit]
            Requires=roscore.service
            PartOf=roscore.service
            After=NetworkManager.service time-sync.target roscore.service
            [Service]
            Type=simple
            User=[TODO enter user name here]
            ExecStart=/usr/sbin/roslaunch
            [Install]
            WantedBy=multi-user.target
        /usr/sbin/roslaunch
            #!/bin/bash
            source ~/catkin_ws/devel/setup.bash
            source /etc/ros/env.sh
            export ROS_HOME=$(echo ~[TODO unter your username here)/.ros
            [TODO place your roslaunch command here] &
            PID=$!
            wait “$PID”

        sudo systemctl enable roscore.service
        sudo systemctl enable roslaunch.service
        sudo chmod +x /usr/sbin/roslaunch

    autologon:
        https://ubuntuqa.com/zh-tw/article/9109.html
    camera calibration：
        refer to Page 450 of effective robtoics programming w/ ros
        pg 312 of mastering ROS for robtoics programming
        save the camera.yaml to /home/roach/.ros/camera_info/
        calibrating kinect:
            pg 404 of ros robotics by example 2nd editoin
     rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=camera/image_raw camera:=/cv_camera

    upgrade to ROS 2.0
     
    use encoder.h in arduino. refer to MIT212 lab's example
    substitute messenger with serial.event
    pi is operating at 3.3v. ardunio like due and teensy also use 3.3v so no voltage level-shifter is required.
    sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target    

    pub battery info
    Use 2 ’s complement arithmetic to subtract previous from current readings to obtain position increment.
    dynamic mpu claibration
    check out joint limit
    calibrate motor ticks and camera
    connect imu's ad0 to ground so it address will be fixed at 0x68

    udev rules for fixing ttyusbX for a usb device
        Check that your rule follows the naming convention – <priority>-<device name>.rules. Technically you can have multiple rules for the same device, and the number determines what order they’d get executed in. Since we’re writing addon rules, a priority of 99 is safest.
        https://blog.csdn.net/qq_16775293/article/details/81332690   
        lsusb to check idVendor and idProduct (in idVendor:idProduct order)
        0. Udev stores all the rules in the /etc/udev/rules.d/ 
        1. connect a device to a usb port, dmesg|grep 
        2. udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0) |grep serial
        3. look for serial and product
        4. ACTION=="add", ATTRS{idVendor}=="", ATTR<idProduct>=="", SYMLINK+="arduion"
        5. uplug your devce and re-plug it in, and check the /dev/arduino
        6. a reboot may require
        =====================================================================
        rosrun turtlebot3_bringup create_udev_rules
        sudo udevadm control --reload-rules && sudo service udev restart && udevadm trigger
    refer to turtlebot setup to enable boot before wifi available
    study vel limitation of h/w interefece 
    tf2:
        sudo apt-get install ros-noetic-tf2-tools
    do i need to broadcast imu?
    rostopic bw /camera/rgb/image color

    // https://www.ross-robotics.co.uk/news/ros-web-tutorial-part-2-working-with-cameras
    // To learn more about our camera we will execute the following command in a terminal:
    v4l2-ctl --list-formats-ext -d /dev/video0

    debgging:
        cv_camer and usb_camera consume about the same cpu resources. choose raw image topic in rviz for fewer resources usage.
        right wheel threshold 45
        why robot won't move?
            1. check /etc/ttyUSB0 & 1
            2. echo imu/data to see if it is published
            3. measure the voltage of batteries output 
            4. if map keep rotating during nav, check odom (ticks, vel)
            5. motor's encoder led must be on

            rosservice call /move_base/clear_costmaps "{}"

            rostopic pub -l /cmd_vel geometry_msgs/Twist -- '[0.3, 0.0, 0.0]' '[0.0, 0.0, 6.283]'

        xacro <xacro file> > tmp.urdf
        check_rudf tmp.urdf
        urdf_to_graphiz tmp.urdf

        git checkout -- *
        git stash 
        git checkout <branch or tag>    
        git checkout master
        git stash pop

permission:
    sudo usermod -aG video $USER
    sudo usermod -aG dialout $USER
        
    
    
