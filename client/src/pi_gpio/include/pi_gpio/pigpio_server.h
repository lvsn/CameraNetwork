#ifndef H_PIGPIO_SERVER
#define H_PIGPIO_SERVER

#include <ros/ros.h>
#include <ros/console.h>

#include <camera_network_msgs/CaptureService.h>
#include <camera_network_msgs/Capture.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#include <wiringPi.h>

#define DEBOUNCE_TIME 200

class gpio_input_handler{
    public:
        gpio_input_handler(ros::NodeHandle nh);
        
        void publishNetworkShot();
        void callShotService();
        unsigned int mTimer[3];
    private:
        ros::NodeHandle mNh;
        ros::ServiceClient mCapture_client; 
        ros::Publisher mChatter_pub; 
};

void setShotInterupt(void);
void setNetworkShotInterupt(void);
void setTimelapsInterupt(void);



#endif
