#!/bin/bash

    echo "Run simulation full system"

    echo "Launching: hub.launch"

    gnome-terminal -- sh -c "roslaunch chefbot_bringup hub.launch; exit"

    echo "Waiting 3s left"

    sleep 3

    echo "Launching: bringup.launch"

    gnome-terminal -- sh -c "roslaunch chefbot_bringup pi.launch; exit"

    echo "Waiting 1s left"

    sleep 1
        