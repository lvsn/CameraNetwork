#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/console.h>

#include <sys/types.h>
#include <sys/stat.h>
<<<<<<< HEAD:src/camera_controler/src/image_streamer.cpp
#include <unistd.h>
=======
/*
Created on Wed May 21 14:59:35 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

(need to be improved) This node only take send.jpg picture in a path and
publish it to streamer It could be done with python directly after picture taken
(it would help for speed)
*/

>>>>>>> 91b6ea2bf0ca81f5d482124072d76c468dca4398:client/src/camera_controler/src/image_streamer.cpp

inline bool exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  char *username = getlogin();
  std::string UserName = username;
  std::string homePath = "/home/"+UserName+"/Images";
  const std::string streamImagePath = homePath + "/preview/send.jpeg";
  ROS_INFO_STREAM("Setting jpeg Streaming path to " << streamImagePath);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/preview", 100);

  ros::Time time = ros::Time::now();
  sensor_msgs::Image im;


  ros::Rate loop_rate(1);
  while (nh.ok()) {
    if ( exists( streamImagePath)){
      cv::Mat cv_image = cv::imread(streamImagePath, CV_LOAD_IMAGE_COLOR); 
      cv_bridge::CvImage cvi;
      cvi.image = cv_image;
      time = ros::Time::now(); 
      cvi.header.stamp = time;
      cvi.header.frame_id = "image";
      cvi.encoding = "bgr8";
  
      cvi.toImageMsg(im);
    
      pub.publish(im);
      remove(streamImagePath.c_str());
      ROS_INFO("Picture Published");
    } 
    ros::spinOnce();
    loop_rate.sleep();
  }
}



