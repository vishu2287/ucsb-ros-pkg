//=================================================================================================
// Copyright (c) 2011, Paul Filitchkin, Brian Satzinger
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the UCSB Robotics Lab nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PAUL FILITCHKIN BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/subscriber.h>

// iRobot Create control
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <iterator>

using namespace std;

//Typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
boost::mutex m;

struct SRanges
{
  double yMin;
  double yMax;
  double zMin;
  double zMax;
  double xRMin;
  double xLMin;
  double xRMax;
  double xLMax;
};

// It seemed too convoluted to use std_msgs::RGBA_ so I added my own struct
struct SRgba
{
  double r;
  double g;
  double b;
  double a;
};

//==================================================================================================
// Description:
//   This callback intercepts the message from the ROS Kinect node
//==================================================================================================
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  m.lock();
  cloud_ = cloud;
  m.unlock();
}

//==================================================================================================
//==================================================================================================
void defaultRanges(struct SRanges& r)
{
  r.xLMin = -1.0f;
  r.xLMax = -0.2f;
  r.xRMin =  0.2f;
  r.xRMax =  1.0f;
  r.yMin  = -0.25f;
  r.yMax  =  0.9f;
  r.zMin  = -1.0f;
  r.zMax  =  4.0f;
}

//==================================================================================================
//==================================================================================================
void readRangeParameters(ros::NodeHandle& nh, struct SRanges& r)
{

  double val;

  if (nh.getParam("/first_test/xl_min", val)) r.xLMin = val;
  if (nh.getParam("/first_test/xl_max", val)) r.xLMax = val;
  if (nh.getParam("/first_test/xr_min", val)) r.xRMax = val;
  if (nh.getParam("/first_test/xr_max", val)) r.xRMax = val;
  if (nh.getParam("/first_test/y_min", val))  r.yMin = val;
  if (nh.getParam("/first_test/y_max", val))  r.yMax = val;
  if (nh.getParam("/first_test/z_min", val))  r.zMin = val;
  if (nh.getParam("/first_test/z_max", val))  r.zMax = val;

}

//==================================================================================================
//==================================================================================================
void publishBoundsMarker(
    ros::Publisher& pub,
    unsigned id,
    const tf::Vector3& pos,
    const tf::Vector3& scale,
    struct SRgba& col)
{

  if (pub.getNumSubscribers() == 0) return;

  visualization_msgs::Marker m;

  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "bounds";
  m.id = id;
  m.type = visualization_msgs::Marker::CUBE;

  m.scale.x = scale.x();
  m.scale.y = scale.y();
  m.scale.z = scale.z();

  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = pos.z();

  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  m.color.r = col.r;
  m.color.g = col.g;
  m.color.b = col.b;
  m.color.a = col.a;

  pub.publish(m);

}

//==================================================================================================
// Main
//==================================================================================================
int main (int argc, char** argv)
{
  ros::init (argc, argv, "first_test");
  ros::NodeHandle nh;
  ros::Publisher velPub; // Create publisher

  SRanges ranges;
  defaultRanges(ranges);

  //Do not send commands to the Create by default
  bool enableCreate = false;

  // The raw Kinect data gets rotated about the x-axis by theta radians
  // This is essentially the measure of tilt between the ground plane and the Kinect
  double theta = -0.3f;

  // This sets the forward speed of the Create
  double speed = 0.1f;

  double val;

  //Used for storing pointcloud from Kinect
  CloudT cloudFull;

  //==================================== ROS Parameters ============================================
  if (nh.getParam("/first_test/enable_create", enableCreate))
  {
    if (enableCreate)
    {
      velPub = nh.advertise<geometry_msgs::Twist>("create_node/cmd_vel", 1);
    }
  }

  // Get the Kinect rotation angle
  if (nh.getParam("/first_test/theta", val)) theta = val;

  // Get the forward speed for the create
  if (nh.getParam("/first_test/speed", val)) speed = val;

  readRangeParameters(nh, ranges);

  // Precomputed for efficiency
  const float cosTheta = cos(theta);
  const float sinTheta = sin(theta);

  //================================= Subscribers/publishers =======================================
  const int queueSize = 1;
  ros::Subscriber sub = nh.subscribe("/kinect/depth/points2", queueSize, cloud_cb);

  ros::Publisher rotatedCloudPub = nh.advertise<sensor_msgs::PointCloud2>("rotated_cloud", 1);
  ros::Publisher fullMarkerPub  = nh.advertise<visualization_msgs::Marker>("full_slice", 1);
  ros::Publisher leftMarkerPub  = nh.advertise<visualization_msgs::Marker>("left_slice", 1);
  ros::Publisher rightMarkerPub = nh.advertise<visualization_msgs::Marker>("right_slice", 1);

  //===================================== Main Loop ================================================
  while (nh.ok())
  {
    // Spin
    ros::spinOnce();
    ros::Duration(0.001).sleep();

    // If no cloud received yet skip everything below
    if (!cloud_) continue;

    // If a new cloud has not been received skip everything below
    if (cloud_ == cloud_old_) continue;

    // A new cloud has been received so convert the ROS message to a PointCloud<T>
    m.lock();
    {
      pcl::fromROSMsg(*cloud_, cloudFull);
    }
    m.unlock();

    cloud_old_ = cloud_;

    CloudT rotatedCloud;
    CloudT leftBlock;
    CloudT rightBlock;

    int leftCnt  = 0;
    int rightCnt = 0;
    int total    = 1; // Prevent divide by zero

    for (CloudT::const_iterator it = cloudFull.begin(); it != cloudFull.end(); ++it)
    {

      // Rotate about the x-axis
      float x = it->x;
      float y = (it->y)*cosTheta - (it->z)*sinTheta;
      float z = (it->y)*sinTheta + (it->z)*cosTheta;

      PointT point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.rgb = it->rgb;

      rotatedCloud.push_back(point);

      // Cut out points above and below vertical threshold
      if ((y > ranges.yMin) && (y < ranges.yMax))
      {

        if ((z > ranges.zMin) && (z < ranges.zMax))
        {
          if ((x < ranges.xLMax) && (x > ranges.xLMin))
          {
            leftBlock.push_back(point);
            leftCnt++;
          }
          
          if ((x < ranges.xRMax) && (x > ranges.xRMin))
          {
            rightBlock.push_back(point);
            rightCnt++;
          }
          total++;
        }
      }
    }

    //===================================== Markers ================================================
    tf::Vector3 pos;
    tf::Vector3 scale;
    struct SRgba col;

    pos.setX(0);
    pos.setY((ranges.yMax - ranges.yMin)/2 + ranges.yMin);
    pos.setZ((ranges.zMax - ranges.zMin)/2 + ranges.zMin);

    scale.setX(4);
    scale.setY(ranges.yMax - ranges.yMin);
    scale.setZ(ranges.zMax - ranges.zMin);

    col.r = 0.0f;
    col.g = 1.0f;
    col.b = 0.0f;
    col.a = 0.3f;

    // Full slice
    publishBoundsMarker(fullMarkerPub, 0, pos, scale, col);

    // Left bounds
    pos.setX((ranges.xLMax - ranges.xLMin)/2 + ranges.xLMin);
    scale.setX(ranges.xLMax - ranges.xLMin);

    col.r = 0.0f;
    col.g = 0.0f;
    col.b = 1.0f;
    col.a = 0.4f;

    publishBoundsMarker(leftMarkerPub, 1, pos, scale, col);

    // Right bounds
    pos.setX((ranges.xRMax - ranges.xRMin)/2 + ranges.xRMin);
    scale.setX(ranges.xRMax - ranges.xRMin);

    col.r = 1.0f;
    col.g = 0.0f;
    col.b = 0.0f;
    col.a = 0.4f;

    publishBoundsMarker(rightMarkerPub, 2, pos, scale, col);

    //=================================== Point Clouds =============================================
    sensor_msgs::PointCloud2 rotatedCloudMsg;

    pcl::toROSMsg(rotatedCloud, rotatedCloudMsg);

    rotatedCloudMsg.header.frame_id = "world";
    rotatedCloudMsg.header.stamp = ros::Time::now();

    rotatedCloudPub.publish(rotatedCloudMsg);

    if (enableCreate)
    {
      geometry_msgs::Twist twist;
      geometry_msgs::Vector3 twistLinear;
      geometry_msgs::Vector3 twistAngular;

      twistLinear.x = speed;
      twistLinear.y = 0;
      twistLinear.z = 0;

      twistAngular.x = 0;
      twistAngular.y = 0;
      twistAngular.z = (float)rightCnt/(float)total - (float)leftCnt/(float)total;

      twist.linear  = twistLinear;
      twist.angular = twistAngular;

      velPub.publish(twist);
    }
  }

  return (0);
}
