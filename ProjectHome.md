## Introduction ##
This Google code project contains open source code developed at the UCSB Robotics Lab using the Robotic Operating System ([ROS.org](http://www.ros.org/)) framework.

---

## Getting started ##
To run the latest code make sure you have a recent version of ROS installed on your system and run the following commands from the command prompt:
```
cd ~
svn checkout https://ucsb-ros-pkg.googlecode.com/svn/trunk/ ucsb-ros-pkg
export ROS_PACKAGE_PATH=:~/ucsb-ros-pkg:$ROS_PACKAGE_PATH
```
_Note: Installing in your home directory is not required so replace "~" with the desired path_

To get started with a specific node please view the one of the node wikis below.

---

## Nodes ##
  * [create\_kinect](create_kinect.md)
  * [create\_webcam](create_webcam.md) - Remote webcam viewing and teleoperation on the iRobot Create
  * create\_joy
  * surf\_object\_recognition

---

## Project Videos ##
<a href='http://www.youtube.com/watch?feature=player_embedded&v=dK6qJgt4niU' target='_blank'><img src='http://img.youtube.com/vi/dK6qJgt4niU/0.jpg' width='425' height=344 /></a>

<a href='http://www.youtube.com/watch?feature=player_embedded&v=pM_MExbug-s' target='_blank'><img src='http://img.youtube.com/vi/pM_MExbug-s/0.jpg' width='425' height=344 /></a>

## UCSB Robotics Lab Create + Kinect Platform ##
For the most part we followed [this](http://www.ros.org/wiki/kinect/Tutorials/Adding%20a%20Kinect%20to%20an%20iRobot%20Create) tutorial.  Instead of a linear regulator we used a switching regulator module, [78SR112HC](http://focus.ti.com/docs/prod/folders/print/78sr112.html) purchased from [DigiKey](http://search.digikey.com/scripts/DkSearch/dksus.dll?Detail&name=78SR112HC-ND).



![http://ucsb-ros-pkg.googlecode.com/svn/wiki/images/createKinectWithLaptop.jpg](http://ucsb-ros-pkg.googlecode.com/svn/wiki/images/createKinectWithLaptop.jpg)

![http://ucsb-ros-pkg.googlecode.com/svn/wiki/images/createKinectDiagram.jpg](http://ucsb-ros-pkg.googlecode.com/svn/wiki/images/createKinectDiagram.jpg)