#include "pi_gpio/pigpio_server.h"



gpio_input_handler* node;

gpio_input_handler::gpio_input_handler(ros::NodeHandle nh){
    this->mNh = nh;
    mTimer[0] = 0;
    mTimer[1] = 0;
    mTimer[2] = 0;
    int err = wiringPiSetupSys();
    if(err == -1){
        ROS_WARN_STREAM("Unable to setup GPIO");
    }
    else{
        ROS_INFO_STREAM("piGPIO node ready, wiringpi init pin :" << digitalRead(4));
    }
    
    //pinMode does nothing in Sys mode but at least it says the pin configuration
    pinMode(4,INPUT);
    pinMode(22,INPUT);
    pinMode(23,INPUT);
    
    //Setup ISRs
    if( wiringPiISR(4,INT_EDGE_SETUP,&setShotInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Shot Interupt!");
    }
    if( wiringPiISR(22,INT_EDGE_SETUP,&setNetworkShotInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Network Shot Interupt!");
    }
    if( wiringPiISR(23,INT_EDGE_SETUP,&setTimelapsInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Timelaps Interupt!");
    }
}

void setShotInterupt(void){
    if (node->mTimer[0] + DEBOUNCE_TIME > millis()){
        return;
    }
    ROS_INFO_STREAM("Button Shot pressed");
    ros::ServiceClient captureClient = node->mNh.serviceClient<camera_network_msgs::CaptureService>("capture_camera");
    camera_network_msgs::CaptureService::Request req;
    camera_network_msgs::CaptureService::Response resp;
    req.time = "heh";
    bool success = captureClient.call(req,resp);
    if (!success)
        ROS_WARN_STREAM("error calling capture_image client");

    node->mTimer[0] = millis();
}

void setNetworkShotInterupt(void){
    if (node->mTimer[1] + DEBOUNCE_TIME > millis()){
        return;
    }
    ROS_INFO_STREAM("Network Shot!!");

    node->mTimer[1] = millis();
}

void setTimelapsInterupt(void){
    if (node->mTimer[2] + DEBOUNCE_TIME > millis()){
        return;
    }
    ROS_INFO_STREAM("Timelaps Shot!!");

    node->mTimer[2] = millis();
}


int main(int argc, char** argv){
    
    ros::init(argc,argv, "pigpio_server");
    ros::NodeHandle nh;
    node = new gpio_input_handler(nh);    
    ros::spin();
    ros::shutdown();
    return 0;
}
