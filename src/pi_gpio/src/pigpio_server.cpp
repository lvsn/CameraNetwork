#include "pi_gpio/pigpio_server.h"



gpio_input_handler* node;

gpio_input_handler::gpio_input_handler(ros::NodeHandle nh){
    this->mNh = nh;
    mTimer[0] = 0;
    mTimer[1] = 0;
    mTimer[2] = 0;

    system("gpio export 4 in");
    system("gpio edge 4 falling");
    system("gpio export 22 in");
    system("gpio edge 22 falling");
    system("gpio export 23 in");
    system("gpio edge 23 falling");
    

    int err = wiringPiSetupSys();
    if(err == -1){
        ROS_WARN_STREAM("Unable to setup GPIO");
    }
    else{
        ROS_INFO_STREAM("piGPIO node ready");
    }
    
    //pinMode does nothing in Sys mode but at least it says the pin configuration
    //pinMode(4,INPUT);
    //pinMode(22,INPUT);
    //pinMode(23,INPUT);
    
    //Setup ISRs
    if( wiringPiISR(4,INT_EDGE_SETUP,&setShotInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Shot Interupt!");
    }
    if( wiringPiISR(22,INT_EDGE_SETUP,&setNetworkShotInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Network Shot Interupt!");
    }
    if( wiringPiISR(22,INT_EDGE_SETUP,&setTimelapsInterupt) < 0){
        ROS_WARN_STREAM("Unable to setup Timelaps Interupt!");
    }

    //Setup publishers 
    mChatter_pub = mNh.advertise<camera_network_msgs::Capture>("/network_capture_chatter", 1);
    mCapture_client = mNh.serviceClient<camera_network_msgs::CaptureService>("capture_camera");
}

void gpio_input_handler::publishNetworkShot(){
    camera_network_msgs::Capture msg;
    msg.isHdr = false;
    mChatter_pub.publish(msg);
}

void gpio_input_handler::callShotService(){
    camera_network_msgs::CaptureService::Request req;
    camera_network_msgs::CaptureService::Response resp;
    req.timer= 0;
    bool success = mCapture_client.call(req,resp);
    if (!success)
        ROS_WARN_STREAM("error calling capture_image client : check if camera's service are online");


}

void setShotInterupt(void){
    if (node->mTimer[0] + DEBOUNCE_TIME > millis()){
        return;
    }
    ROS_INFO_STREAM("Button Shot pressed");
    node->callShotService();
    node->mTimer[0] = millis();
}

void setNetworkShotInterupt(void){
    if (node->mTimer[1] + DEBOUNCE_TIME > millis()){
        return;
    }
    ROS_INFO_STREAM("Button network Shot pressed");
    node->publishNetworkShot();
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
