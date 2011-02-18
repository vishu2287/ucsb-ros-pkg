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

#include <iostream>
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

/*
#define XLMIN -1.0f
#define XLMAX -0.2f
#define XRMIN  0.2f
#define XRMAX  1.0f
#define YMIN -0.25f
#define YMAX   0.9f
#define ZMIN  -1.0f
#define ZMAX   1.0f
*/

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

  if (nh.getParam("/first_test/theta", val)) theta = val;

  readRangeParameters(nh, ranges);

  // Precomputed for efficiency
  const float cosTheta = cos(theta);
  const float sinTheta = sin(theta);

  //================================= Subscribers/publishers =======================================
  const int queue_size = 1;
  ros::Subscriber sub = nh.subscribe("/kinect/depth/points2", queue_size, cloud_cb);

  ros::Publisher sliceCloudPub = nh.advertise<sensor_msgs::PointCloud2>("slice_cloud", 1);
  ros::Publisher leftCloudPub  = nh.advertise<sensor_msgs::PointCloud2>("left_cloud", 1);
  ros::Publisher rightCloudPub = nh.advertise<sensor_msgs::PointCloud2>("right_cloud", 1);

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

    CloudT fullSlice;
    CloudT leftBlock;
    CloudT rightBlock;

    int leftCnt  = 0;
    int rightCnt = 0;
    int total    = 0;

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

      fullSlice.push_back(point);

      // Cut out points above and below vertical threshold
      if ((y > ranges.yMin) && (y < ranges.yMax))
      {
        if ((z > ranges.zMin) && (z < ranges.zMax))
        {

          if ((x < ranges.xLMin) && (x > ranges.xLMax))
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


    // Full slice
    visualization_msgs::Marker fullSliceMarker;

    fullSliceMarker.header.frame_id = "kinect_depth";
    fullSliceMarker.header.stamp = ros::Time::now();
    fullSliceMarker.ns = "debug";
    fullSliceMarker.id = 0;
    fullSliceMarker.type = visualization_msgs::Marker::CUBE;

    fullSliceMarker.scale.x = 10;
    fullSliceMarker.scale.y = ranges.yMax - ranges.yMin;
    fullSliceMarker.scale.z = ranges.zMax - ranges.zMin;

    fullSliceMarker.pose.position.x = 0;
    fullSliceMarker.pose.position.y = (ranges.yMax - ranges.yMin)/2 + ranges.yMin;
    fullSliceMarker.pose.position.z = (ranges.zMax - ranges.zMin)/2 + ranges.zMin;

    fullSliceMarker.pose.orientation.x = 0.0;
    fullSliceMarker.pose.orientation.y = 0.0;
    fullSliceMarker.pose.orientation.z = 0.0;
    fullSliceMarker.pose.orientation.w = 1.0;

    fullSliceMarker.color.r = 0.0f;
    fullSliceMarker.color.g = 1.0f;
    fullSliceMarker.color.b = 0.0f;
    fullSliceMarker.color.a = 0.4;

    fullMarkerPub.publish(fullSliceMarker);


    // Left slice
    visualization_msgs::Marker leftSliceMarker;

    leftSliceMarker.header.frame_id = "kinect_depth";
    leftSliceMarker.header.stamp = ros::Time::now();
    leftSliceMarker.ns = "debug";
    leftSliceMarker.id = 1;
    leftSliceMarker.type = visualization_msgs::Marker::CUBE;

    leftSliceMarker.scale.x = ranges.xLMax - ranges.xLMin;
    leftSliceMarker.scale.y = ranges.yMax - ranges.yMin;
    leftSliceMarker.scale.z = ranges.zMax - ranges.zMin;

    leftSliceMarker.pose.position.x = (ranges.xLMax - ranges.xLMin)/2 + ranges.xLMin;
    leftSliceMarker.pose.position.y = (ranges.yMax - ranges.yMin)/2 + ranges.yMin;
    leftSliceMarker.pose.position.z = (ranges.zMax - ranges.zMin)/2 + ranges.zMin;

    leftSliceMarker.pose.orientation.x = 0.0;
    leftSliceMarker.pose.orientation.y = 0.0;
    leftSliceMarker.pose.orientation.z = 0.0;
    leftSliceMarker.pose.orientation.w = 1.0;

    leftSliceMarker.color.r = 1.0f;
    leftSliceMarker.color.g = 0.0f;
    leftSliceMarker.color.b = 0.0f;
    leftSliceMarker.color.a = 0.4;

    leftMarkerPub.publish(leftSliceMarker);



    // Right slice
    visualization_msgs::Marker rightSliceMarker;

    rightSliceMarker.header.frame_id = "kinect_depth";
    rightSliceMarker.header.stamp = ros::Time::now();
    rightSliceMarker.ns = "debug";
    rightSliceMarker.id = 2;
    rightSliceMarker.type = visualization_msgs::Marker::CUBE;

    rightSliceMarker.scale.x = ranges.xRMax - ranges.xRMin;
    rightSliceMarker.scale.y = ranges.yMax - ranges.yMin;
    rightSliceMarker.scale.z = ranges.zMax - ranges.zMin;

    rightSliceMarker.pose.position.x = (ranges.xRMax - ranges.xRMin)/2 + ranges.xRMin;
    rightSliceMarker.pose.position.y = (ranges.yMax - ranges.yMin)/2 + ranges.yMin;
    rightSliceMarker.pose.position.z = (ranges.zMax - ranges.zMin)/2 + ranges.zMin;

    rightSliceMarker.pose.orientation.x = 0.0;
    rightSliceMarker.pose.orientation.y = 0.0;
    rightSliceMarker.pose.orientation.z = 0.0;
    rightSliceMarker.pose.orientation.w = 1.0;

    rightSliceMarker.color.r = 0.0f;
    rightSliceMarker.color.g = 0.0f;
    rightSliceMarker.color.b = 1.0f;
    rightSliceMarker.color.a = 0.4;

    rightMarkerPub.publish(rightSliceMarker);



    sensor_msgs::PointCloud2 sliceCloudMsg;
    sensor_msgs::PointCloud2 leftCloudMsg;
    sensor_msgs::PointCloud2 rightCloudMsg;

    pcl::toROSMsg(fullSlice, sliceCloudMsg);
    pcl::toROSMsg(leftBlock, leftCloudMsg);
    pcl::toROSMsg(rightBlock, rightCloudMsg);

    sliceCloudMsg.header.frame_id = "kinect_depth";
    sliceCloudMsg.header.stamp = ros::Time::now();

    leftCloudMsg.header.frame_id = "kinect_depth";
    leftCloudMsg.header.stamp = ros::Time::now();

    rightCloudMsg.header.frame_id = "kinect_depth";
    rightCloudMsg.header.stamp = ros::Time::now();

    sliceCloudPub.publish(sliceCloudMsg);
    leftCloudPub.publish(leftCloudMsg);
    rightCloudPub.publish(rightCloudMsg);

    if (enableCreate)
    {
      float rFudgeFactor = 1.0f;
      float lFudgeFactor = rFudgeFactor;

      geometry_msgs::Twist twist;
      geometry_msgs::Vector3 twistLinear;
      geometry_msgs::Vector3 twistAngular;
      twistLinear.x = 0.1;
      twistLinear.y = 0;
      twistLinear.z = 0;

      twistAngular.x = 0;
      twistAngular.y = 0;
      twistAngular.z =
          rFudgeFactor*(float)rightCnt/(float)total - lFudgeFactor*(float)leftCnt/(float)total;

      twist.linear  = twistLinear;
      twist.angular = twistAngular;

      velPub.publish(twist);
    }
  }

  return (0);
}
