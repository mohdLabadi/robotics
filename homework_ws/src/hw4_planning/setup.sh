#!/bin/bash

## HW4 Setup Script.
## Places the HW4 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The homework_4 repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.
## - Your workspace structure follows the pre-HW instructions.

cd ~/Downloads
unzip homework_4.zip
mv ~/Downloads/homework_4/hw4_planning/ ~/homework_ws/src/
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash
