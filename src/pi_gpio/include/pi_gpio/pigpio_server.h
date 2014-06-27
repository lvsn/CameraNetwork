#ifndef H_PIGPIO_SERVER
#define H_PIGPIO_SERVER

#include <ros/ros.h>
#include <ros/console.h>

#include <camera_network_msgs/CaptureService.h>

#include <iostream>
#include <string>

#include <wiringPi.h>

#define DEBOUNCE_TIME 200

class gpio_input_handler{
    public:
        gpio_input_handler(ros::NodeHandle nh);
    
        unsigned int mTimer[3];
        ros::NodeHandle mNh;
};

void setShotInterupt(void);
void setNetworkShotInterupt(void);
void setTimelapsInterupt(void);



#endif
