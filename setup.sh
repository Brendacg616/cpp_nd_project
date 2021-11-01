#! /bin/bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install libsdl2-ttf-dev ros-kinetic-ros-controllers -y
pip install pyyaml 
cd /home/workspace/catkin_ws/
catkin_make
source /home/workspace/catkin_ws/devel/setup.bash