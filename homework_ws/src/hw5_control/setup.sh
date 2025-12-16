#!/bin/bash

## HW5 + Final Project Setup Script.
## Places the HW5 + Final Project code into a NEW ROS workspace ~/finalhomework_ws and builds the packages.

## Preconditions:
## - The cs4750_student repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The NEW ~/finalhomework_ws/src folder has been created.

cd ~/Downloads
unzip finalhomework_ws-main.zip
cp -r finalhomework_ws-main/ ~/finalhomework_ws
cd ~
cd finalhomework_ws
catkin build
