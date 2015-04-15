## Description ##

This package enables users to remotely view/transmit webcam images and control an iRobot Create with a joystick.  Webcam images can be viewed locally or they can be compressed and transmitted over the network.  A teleoperation node is included to provide joystick control using the create\_node package.

## Dependencies ##
  * [create\_node](http://www.ros.org/wiki/create_node)
  * [image\_transport](http://www.ros.org/wiki/image_transport)
  * [opencv2](http://www.ros.org/wiki/opencv2)
  * [cv\_bridge](http://www.ros.org/wiki/cv_bridge)
  * [create\_node](http://www.ros.org/browse/details.php?name=create_node)
  * [joy](http://www.ros.org/wiki/joy)

## Getting started ##
```
roscd create_webcam
roslaunch ./launch/localAll.launch
```

## Launch files ##
  * **hostAll.launch** - Launch an image subscriber and joystick controller on the host machine
  * **hostWebcamView.launch** - Launch an image subscriber for viewing compressed webcam images on the host machine
  * **localAll.launch** - Test compressed image transmission and teleoperation on the same machine
  * **localImageNoCompress.launch** - Publish an image with no compression and subscriber to it locally
  * **localWebcamCompress.launch** - Publish webcam images with compression and subscribe to them locally
  * **localWebcamNoCompress.launch** - Publish webcam images without compression and subscribe to it locally
  * **robotAll.launch** - Launch all of the necessary nodes for remote webcam viewing and teleoperation on the robot platform
  * **robotWebcam.launch** - Launch a compressed image publisher on the robot