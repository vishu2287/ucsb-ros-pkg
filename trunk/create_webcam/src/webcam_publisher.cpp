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
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>

using namespace cv;
using namespace std;

//=================================================================================================
//=================================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  VideoCapture cap(0); // open the default camera

  const unsigned int CAPTURE_WIDTH = 640;
  const unsigned int CAPTURE_HEIGHT = 480;
  const string& cvOutputWindowName = "CV Output";  

  cap.set(CV_CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT);

  // check if we succeeded
  if ( !cap.isOpened() )
  {
    cout << "Could not open capture device\n";
    return -1;
  }

  Mat frame;

  ros::Rate loop_rate(100);

  sensor_msgs::ImagePtr msg;

  while (nh.ok())
  {
    cap >> frame; // get a new frame from camera

    IplImage frm = _IplImage(frame); //Unfortunately must convert to old format
    msg = sensor_msgs::CvBridge::cvToImgMsg(&frm, "bgr8");

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
