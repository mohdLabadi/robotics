#!/bin/bash

## HW2 Setup Script.
## Places the HW2 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The homework_2 repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip homework_2-main.zip
mv ~/Downloads/homework_2-main/hw2_kinematics/ ~/homework_ws/src/cs4750_student
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash