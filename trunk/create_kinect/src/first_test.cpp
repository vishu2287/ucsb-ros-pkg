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
#include <pcl_visualization/pcl_visualizer.h>

#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>

// iRobot Create control
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>
#include <iterator>

using namespace std;
using terminal_tools::print_highlight;
using terminal_tools::parse_argument;

typedef pcl::PointXYZ Point;
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
boost::mutex m;

#define YMIN -0.25f
#define YMAX   0.9f
#define XLMIN -1.0f
#define XLMAX -0.2f
#define XRMIN  0.2f
#define XRMAX  1.0f
#define ZMIN   -1.0f
#define ZMAX   1.0f

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  /*
  ROS_INFO ("PointCloud with %d data points (%s), stamp %f, and frame %s.",
             cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), 
             cloud->header.stamp.toSec (), cloud->header.frame_id.c_str ());
  */
  m.lock();
  cloud_ = cloud;
  m.unlock();
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "first_test");
  ros::NodeHandle nh;
  ros::Publisher velPub; // Create publisher

  //Do not send commands to the Create by default
  bool driveCreate = false;
  bool pclVisualizer = false;

  if (nh.getParam("/first_test/drive_create", driveCreate))
  {
    if (driveCreate)
    {
      velPub = nh.advertise<geometry_msgs::Twist>("create_node/cmd_vel", 1);
    }
  }

  /*
  if (nh.getParam("/first_test/pcl_visualizer", pclVisualizer))
  {
    if (pclVisualizer)
    {
    }
  }
  */

  pcl_visualization::PCLVisualizer p(argc, argv, "create_kinect_nav_viewer");

  // Get the queue size from the command line
  int queue_size = 1;
  parse_argument (argc, argv, "-qsize", queue_size);
  print_highlight ("Using a queue size of %d\n", queue_size);

  // Get the number of clouds to keep on screen
  int nr_clouds = 1;
  parse_argument (argc, argv, "-nclouds", nr_clouds);

  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe ("/kinect/depth/points2", queue_size, cloud_cb);



  pcl::PointCloud<Point> cloud_xyz;
  ColorHandlerPtr color_handler;

  while (nh.ok())
  {
    // Spin
    ros::spinOnce();
    ros::Duration(0.001).sleep();
    p.spinOnce(10);

    // If no cloud received yet, continue
    if (!cloud_)
      continue;

    if (cloud_ == cloud_old_)
      continue;

    p.removePointCloud("cloud");

    // Convert to PointCloud<T>
    m.lock();
    {
      pcl::fromROSMsg (*cloud_, cloud_xyz);

      pcl::PointCloud<Point> fullSlice;
      pcl::PointCloud<Point> leftBlock;
      pcl::PointCloud<Point> rightBlock;

      int leftCnt = 0;
      int rightCnt = 0;
      int total = 0;

      for (pcl::PointCloud<Point>::const_iterator it = cloud_xyz.begin();
          it != cloud_xyz.end(); ++it)
      {
        float x = (*it).x;
        float y = (*it).y;
        float z = (*it).z;

        // Cut out points above and below vertical threshold
        if ((y < YMAX) && (y > YMIN))
        {
          if ((z < ZMAX) && (z > ZMIN))
          {
            if ((x < XLMAX) && (x > XLMIN))
            {
              leftBlock.push_back(*it);
              leftCnt++;
            }
            
            if ((x < XRMAX) && (x > XRMIN))
            {
              rightBlock.push_back(*it);
              rightCnt++;
            }
          }
          
          fullSlice.push_back(*it);
          total++;
        }
      }

      //stringstream ss;
      //ss << "left: " << leftCnt << " right: " << rightCnt << " total: " << total << "\n";
      //ROS_INFO(ss.str().c_str());

      if (driveCreate)
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

      /*
      NEED TO USE THIS BUT NOT ENOUGH TIME TO UNDERSTAND pcl::PointIndices
      
      Eigen3::Vector4f& minPoint = Eigen3::Vector4f();
      Eigen3::Vector4f& maxPoint = Eigen3::Vector4f();
      
      pcl::getMinMax3D( const pcl::PointCloud< PointT > & cloud, 
      const pcl::PointIndices & indices, 
      minPoint, 
      maxPoint);
      */

      sensor_msgs::PointCloud2 displayCloud;

      pcl::toROSMsg((const pcl::PointCloud<Point>) fullSlice, displayCloud);

      // Set the colorhandler to use the rgb fields from the kinect
      color_handler.reset(
        new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> 
        (displayCloud));

      p.addPointCloud(fullSlice, color_handler, "cloud");

      // Add a color handler for each field
      for (size_t i = 0; i < displayCloud.fields.size(); ++i)
      {
        color_handler.reset(
          new pcl_visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> 
          (displayCloud, displayCloud.fields[i].name));
          
        p.addPointCloud( fullSlice, color_handler, "cloud" );
      }
      cloud_old_ = cloud_;
    }
    m.unlock();

    //Add the cloud to the renderer
    //ROS_INFO ("New cloud added to the renderer!");
  }

  return (0);
}
