## Description ##

This first test implements a simple bump bot where the Create drives forward while avoiding obstacles.  Much more to come from this project.

---

<a href='http://www.youtube.com/watch?feature=player_embedded&v=pM_MExbug-s' target='_blank'><img src='http://img.youtube.com/vi/pM_MExbug-s/0.jpg' width='425' height=344 /></a>

---

## Dependencies ##
  * [create\_node](http://www.ros.org/browse/details.php?name=create_node)
  * [std\_msgs](http://www.ros.org/wiki/std_msgs)
  * [roscpp](http://www.ros.org/wiki/roscpp)
  * [visualization\_msgs](http://www.ros.org/wiki/visualization_msgs)
  * [pcl](http://www.ros.org/wiki/pcl)
  * [geometry\_msgs](http://www.ros.org/wiki/geometry_msgs)

## Getting started ##
This code is intended for the UCSB Robotics Lab Create + Kinect robot platform (or similar).  It is recommended that you install the full ROS Diamondback stack as instructed [here](http://www.ros.org/wiki/diamondback/Installation/Ubuntu) for Ubuntu.
```
sudo apt-get install ros-diamondback-desktop-full
```
In addition you will need the joystick-drivers stack
```
sudo apt-get install ros-diamondback-joystick-drivers
```
And the OpenNI Kinect drivers
```
sudo apt-get install ros-diamondback-openni-kinect
```
The ROS [Kinect page](http://www.ros.org/wiki/kinect) is an excellent starting point and the [OpenNI camera page](http://www.ros.org/wiki/openni_camera) explains a lot of useful features.  To install the iRobot Create drivers run the following commands
```
cd ~
svn co https://code.ros.org/svn/ros-pkg/branches/trunk_cturtle/stacks/create_robot/ create_robot
export ROS_PACKAGE_PATH=:~/create_robot:$ROS_PACKAGE_PATH
```
To run this package locally ensure that the Create and Kinect are connected and in a prompt run
```
roslaunch openni_camera openni_node.launch
```
You should see a print statement _[/openni\_node1] Number devices connected: 1_.  After the Kinect drivers are running run
```
roscd create_kinect
roslaunch ./launch/firstTest.launch
```

Now on a separate machine on the same network set the ROS\_MASTER\_URI environment variable accordingly (see [this](http://www.ros.org/wiki/ROS/Tutorials/MultipleMachines) for additional details)
```
roscd create_kinect
roslaunch ./launch/remoteMonitor.launch
```
The remote monitor is used for switching between autonomous and manual mode as well as stopping the robot when needed.

## Launch files ##
  * **firstTest.launch** - Launch a basic bump bot first test with the Kinect mounted on the Create