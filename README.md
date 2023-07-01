# cpp_nd_project
## Project Description

## Workspace configuration
The configuration described below works for Udacity Desktop Workspace. 
### ROS workspace setup
Create a ROS Workspace by running the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source /home/workspace/catkin_ws/devel/bash.sh
```
Clone this repo inside src folder
```
cd ~/catkin_ws/src
git clone https://github.com/Brendacg616/cpp_nd_project.git
```
### Install required packages
This project needs SDL Library and ros_controllers package.  You can run project_setup.sh to install these packages.
```
cd ~/catkin_ws/src/cpp_nd_project
sudo chmod +x setup.sh
./setup.sh
```
#### Compile
```
cd ~/catkin_ws/
catkin_make
```
Now you're ready to run the project.
## Running the project
