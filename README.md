Description
====================
This repository has code for using daVinci research kit on Gazebo 7 and controlled using ROS.

# Author

Ankur Agrawal:asagrawal@wpi.edu

Radian Azhar Gondokaryono:ragondokaryono@wpi.edu


# Install
* download & compile dvrk_gazebo_ros

```sh
# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src
# clone repo
git clone https://github.com/ankur-agrawal/Cart_PSM_ECM.git
# build
cd ..
catkin_make
```

# Dependencies

Gazebo 7, ROS indigo or ROS kinetic. If ROS-indigo is to be used with Gazebo 7, keep gazebo_ros_pksgs (https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel) in your src folder.
