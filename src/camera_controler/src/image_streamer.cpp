#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/console.h>

#include <sys/types.h>
#include <sys/stat.h>

inline bool exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  std::string homePath = getenv ("HOME");
  const std::string streamImagePath = homePath + "/CameraPicture/preview/send.jpeg";
  ROS_INFO_STREAM("Setting jpeg Streaming path to " << streamImagePath);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/preview", 1);

  ros::Time time = ros::Time::now();
  sensor_msgs::Image im;

  //cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );

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



