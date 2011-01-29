//=================================================================================================
// Copyright (c) 2011, Paul Filitchkin
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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include "surfDatabase.h"
#include "surfEntry.h"
#include "ipoint.h"

CSurfDatabase* mpTrainDb;
//=================================================================================================
//=================================================================================================
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;

  try
  {
    IplImage* myImg = bridge.imgMsgToCv(msg, "bgr8");
    CvPoint myPnt;
    myPnt.x = 10;
    myPnt.y = 100;
    CvScalar myClr = {0xFF, 0xFF, 0xFF, 0xFF};
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0, 1.0, 0, 2);

    CSurfEntry myEntry;
    CSurfEntry::SSurfParams params;
    params.mInitSample = 2;
    params.mIntervals  = 4;
    params.mThreshold  = 0.002;
    params.mUpright    = false;
    params.mOctaves    = 5;
    myEntry.Populate(myImg, 0, params, "unknown");

    unsigned matchLabelId = 0;
    unsigned matchCount = 0;
    std::string matchLabel;

    if (mpTrainDb->MatchEntryNN(myEntry, matchLabelId, matchCount))
    {
      matchLabel = mpTrainDb->GetLabelName(matchLabelId);
    }
    else
    {
      matchLabel = "no match!";
    }

    cvPutText(myImg, matchLabel.c_str(), myPnt, &font, myClr);

    drawIpoints(myImg, myEntry.GetDescriptor());
    drawFPS(myImg);
    cvShowImage("view", myImg);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//=================================================================================================
//=================================================================================================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "surf_subscriber");
  ros::NodeHandle nh;

  // Get the SURF database path information
  CSurfDatabase::SPaths paths;

  if (!nh.getParam("/compressed_listener/setup_path", paths.mSetupPath))
  {
    ROS_ERROR("Must set '/compressed_listener/setup_path' parameter!");
  }
  if (!nh.getParam("/compressed_listener/image_path", paths.mImagePath))
  {
    ROS_ERROR("Must set '/compressed_listener/image_path' parameter!");
  }
  if (!nh.getParam("/compressed_listener/database_path", paths.mDatabasePath))
  {
    ROS_ERROR("Must set '/compressed_listener/database_path' parameter!");
  }
  if (!nh.getParam("/compressed_listener/html_path", paths.mHtmlPath))
  {
    ROS_ERROR("Must set '/compressed_listener/html_path' parameter!");
  }

  mpTrainDb = new CSurfDatabase("train", paths, true);

  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("view");
}
