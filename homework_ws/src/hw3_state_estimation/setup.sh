#!/bin/bash

## HW3 Setup Script.
## Places the HW3 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The hw3_state_estimation repo was downloaded as a Zip file from GitHub to ~/Downloads
## - If the zip file is named differently in your Downloads folder, modify the commands accordingly.
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip hw3_state_estimation.zip
mv ~/Downloads/hw3_state_estimation ~/homework_ws/src
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash
roscd arm_state_estimation/