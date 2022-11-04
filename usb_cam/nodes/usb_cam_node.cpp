/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
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
*   * Neither the name of the Robert Bosch nor the names of its
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
*
*********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include<std_msgs/UInt32.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <cstring>
#include <stdlib.h>


namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // 임의로 추가함
  ros::Publisher pub_;
  ros::Subscriber sub_;
  FILE* fp;


  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;
  std::string angleMessage = "angle_topic";
  std::string fileName = "cam1";
  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;


  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {

    std::cout << "\tstart usb cam node\t\n";
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);
    node_.param("angleMessage", angleMessage, std::string("angle_topic"));
    // 임의로 추가함

    node_.param("fileName", fileName, std::string("cam1"));
    const char *file = fileName.c_str();
    fp = fopen(file, "a+");
    sub_ = node_.subscribe("/os_node/" + angleMessage, 1, &UsbCamNode::angleCallbackfunc, this);

    pub_ = node_.advertise<std_msgs::UInt32>("time_gap", 1); // To send time gap between lidar and camera to os_node

    image_transport::ImageTransport image_trans(node_);
    image_transport::Publisher image_pub = image_trans.advertise("/usb_cam/image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 1280);
    node_.param("image_height", image_height_, 760);
    node_.param("framerate", framerate_, 10);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.stamp = ros::Time::now();
      img_.header.stamp = ros::Time::now();
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();

    // 임의로 추가
    fclose(fp);
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }

  //


  // 이하의 angleCallbackfunc은 topic을 subscribe할 때 이를 저장하여 비교하기 위한 함수임
  void angleCallbackfunc(const std_msgs::UInt32::ConstPtr& my_pos) {
	  static int my_count = 0;
	      //ROS_INFO("I heard the message : [%s]", my_pos->data.c_str());
	  // grab the image

      cam_.grab_image(&img_);
	  // grab the camera info
	  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
	  ci->header.frame_id = img_.header.frame_id;
	  unsigned int seconds = ci->header.stamp.nsec;
	  fprintf(fp, "%d\t%d\n", my_pos->data, seconds);

	  // publish the image
	  image_pub_.publish(img_, *ci);

	  // send time_gap topic!!
	  std_msgs::UInt32 time_gap;

	  if(seconds < my_pos->data){

		  seconds = seconds + 10000000; //변경 필요!!! 정확히 얼마를 더해야되는지
	  }
	  unsigned int gap = seconds - (my_pos->data);

	  time_gap.data = gap;
	  pub_.publish(time_gap);

                                                      // subscribe할 당시의 ROSTIME
	      //FILE* fp = fopen("/home/vialab/jjh_ouster_ws/cam_output.txt", "a+");                         // 데이터를 txt파일로 저장하기 위한 코드로써, fopen("파일경로", "openmode")로 구성됨

	      //node를 추가로 안쓰고 하는 방법은 일단 포기 (unrecognized image encoding[])

	      //cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_);

	      //cv_img->encoding = "bgr8";
	      //cv::imwrite("/home/vialab/jjh_ouster_ws/camera_image.jpeg", cv_img->image);


	      //if(my_count == 0) fprintf(fp, "index\tRostimeOfLidar(nsec)\tRostimeOfCamera(nsec)\n");        // 이때, openmode는 a+(append)로 하여야 기존의 기록 아래로 계속 작성하며
	      //fprintf(fp, "%d\t%s\t%llu\n", my_count, my_pos->data.c_str(), ros::Time::now().toNSec());     // w+로 두는 경우 마지막 데이터만 기록될 것이므로 주의가 필요함

	       // 현재는 msg여부와 상관없이 image_raw를 받으면 capture.py에서 capture가 되지만
	      // image_cap_raw를 publish해서 capture하도록 변경 필요


	      //image_transport::CameraPublisher image_pub_;
	      //image_pub_ = it.advertiseCamera("image_raw", 1);

	      //fclose(fp);
	      my_count++;                                                                             // 이 행부터 아래로 4행까지는 lidar가 publish한 데이터와 동일한지
	      std::cout << "ros::Time::now().nsec : " << std::endl;               // 확인하기 위한 검증과정임
	      std::cout << "n : " << my_count << std::endl;
	      std::cout << "====================================\n";
	      ros::spinOnce();

	      /*std::cout << "===================\n" << std::endl;
	      std::cout << my_pos->data.c_str() << std::endl;
	      std::cout << ros::Time::now() << std::endl;
	      std::cout << "===================\n" << std::endl;
	    */
  }



};

}


/*
 void imageCallbackfunc(const sensor_msgs::Image& msg) {
   ROS_INFO("image Callback Function\n");
 }

sensor_msgs::Image temp_img;

*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  //ros::NodeHandle nh;
  //ros::Subscriber sub_angle = nh.subscribe("/os1_node/angle_topic", 1, angleCallbackfunc); // os1_node에서 publish한 topic을 subscribe하는 경우, callbackfunc을 호출함
  //image_transport::ImageTransport image_trans(nh);
  //image_transport::Publisher image_pub = image_trans.advertise("/usb_cam/image_cap_raw", 1);


  /*
  cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
  cv::imwrite("/home/vialab/jjh_ouster_ws/camera_image.jpeg", image);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
*/
  //image_pub.publish(msg);
  ros::spin();
  
  //temp_img = a.img_;  //occur "select timeout" error
  //a.spin();
  
  // ros::spin은 뭐고 a.spin은 뭐지??

  return EXIT_SUCCESS;
}
