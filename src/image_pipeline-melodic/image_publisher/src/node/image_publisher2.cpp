/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/* 1920x1080
camera_mat = (367.2867502156721, 0.0, 960.8303932411943,0.0, 369.06857590098275, 562.6211777029157, 0.0, 0.0, 1.0)
dist_coeff = (0.05820771148994614, 0.011886625709185384, -0.024279444196835236, 0.006027933828260825)*/

// 3264x2448
camera_mat = (367.0543074903704, 0.0, 990.6330750325744, 0.0, 366.7370079611347, 575.1183044201284, 0.0, 0.0, 1.0)
dist_coeff = (0.06062150734934245, -0.008279891153400248, -0.0012545281813805395, -0.0010038515782001421)

cv::VideoCapture cap(1);
if(!cap.isOpened()) 
  {
      ROS_INFO("can not opencv video device\n");
      return 1;
  }
(cap.set(cv::CAP_PROP_FPS, 20))
(cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920))
(cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080))
(cap.set(cv::CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G')))
(cap.set(cv::CAP_PROP_GAIN, 0.4))
(cap.set(cv::CAP_PROP_BRIGHTNESS, 0))
(cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25))
(cap.set(cv::CAP_PROP_EXPOSURE, 0.1))

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image2_publisher", ros::init_options::AnonymousName);

  if (argc <= 1) {
    ROS_ERROR("image_publisher requires filename. Typical command-line usage:\n"
              "\t$ rosrun image_publisher image_publisher <filename>");
    return 1;
  }

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
 
  cv::Mat image;
  cap >> image;
  //cv::imshow("",image);
  cv::Mat camera_matrixa = (cv::Mat_<double>(3, 3) << camera_mat);
  cv::Mat distortion_coefficientsa=(cv::Mat_<double >(1,4)<<dist_coeff);
  cv::fisheye::undistortImage(image, distortion_image, camera_matrixa, distortion_coefficientsa,camera_matrixa,img_sizea);
  cv::imshow("",distortion_image)
  waitKey(0)

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distortion_image).toImageMsg();
 
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spin();
  }

  return 0;
}
