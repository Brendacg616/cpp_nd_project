# cpp_nd_project
##Pre-requisites 

###ROS distro
kinetic 
###Install SDL library
install SDL_ttf sudo apt-get install libsdl2-ttf-dev 
###Install ros_controllers
sudo apt install ros-kinetic-ros-controllers 
##Set up
###Create a workspace
If it doesn't exist yet, create a new catkin workspace 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source /home/workspace/catkin_ws/devel/bash.sh
```
