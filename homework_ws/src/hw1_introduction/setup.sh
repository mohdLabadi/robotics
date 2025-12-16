#!/bin/bash

## HW1 Setup Script.
## Places the HW1 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The cs4750_student repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip cs4750_student-main.zip
cd ~
mkdir -p homework_ws/src
mv ~/Downloads/cs4750_student-main/ ~/homework_ws/src/cs4750_student
cd ~/homework_ws
catkin build 
source ~/homework_ws/devel/setup.bash 