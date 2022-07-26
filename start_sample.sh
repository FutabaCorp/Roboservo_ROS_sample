#!/bin/bash

#Set CAN interface
sudo ip link set can0 up type can bitrate 1000000

#Start sample code
roslaunch roboservo_sample setup.launch
