Camera Network is a camera sensor project developped with ROS (Robotic Operating System) to communicate easily with multiple camera device on the network.

For now, Camera Network is compatible with gphoto and the picamera module.

You can communicate with the network directly with ROS, or with the WebGUI (if the master-server is launched with the right webserver url)

The system is mainly used for still pictures. The ROS interface is made of two abstraction layers : Camera's driver and Camera controler.

#Camera Driver

This layer interact directly with the camera, it is used by camera controler so it need to implement right services to make it compatible with the system. The two actual drivers are : gphoto and picamera. USB could be supported in the futur.

The services are :
* capture_camera : service that take picture and store it in default place
* capture_video : service that take video and store it in default place
* load_camera : service that upload default's picture place to user's place it place the file and rename it with a standard
* get_camera : service that return camera's data (iso, aperture, shutterspeed, format)
* set_camera : service that set camera's data (iso, aperture, shutterspeed, format)
* calibrate_picture : methode that automatically calibrate camera's parameters

This layer is normally not launched directly by the user. Camera controler's launcher will do it


#Camera Controler

This layer interact with the Camera drivers. It can handle network task and individual task. The user should never call camera driver services (ROS give no protection although) but always call Camera Controler. The webGUI is entirely communicating with Camera Controler, so it dont care what kind of camera each device are using. **The button with (beta) mean that it interact with the device, using it wrongly could lead to errors**

To launch the controler you need to make a ROS launchfile with these steps (use the one in this repo as templates):
- Your computer must have the env variable CAMERA_NAME to set a unique namespace
- The launch file must set a parameter in /IP wich is the interface's IP adress
- It must load a yaml file (check template timelaps_nikon.yaml)
- It must load a camera driver (gphoto or picam), camera controler, and image_streamer.
- You can load pigpio if you want to use the button interface

Here is the services,actions and subscribe the camera Controler offer:
Subscriber:
* network_capture_chatter : Publish on this topic to make all device take a picture
* network_capture_video_chatter : Publish on this topic to make all device take a video

Services:
* preview_camera : take a picture and stream it to mjpeg to view (the picture is deleted afterward)
* shutdown_device : can shutdown or reboot the device
* calibrate_device : call Camera's Driver service to calibrate and update datas
* save_config: Save HDR configurations to default param file

Action:
* timelaps : Take picture with parameters : quantity, delay between each pictures and if HDR or normal pictures

#Camera Master

Camera Master is an interface to control the whole network at once with extra network features:
Actions:
* sftp : open sftp session with every device on the network, it download every file called by load_camera
* network_timelaps: Implement the same action as Camera Controler but from the network point of view. Instead of calling devices individually, it publish on topic 

  sftp_node maintain a list of user to log into every device (not safe!) it use user:pi password:raspberry if it cant   find it in the list
  Services:
  * add_user: add new user to list
  * delete_users: remove all users from list
  * save_users: write to default list file
  * get_users: return the entire list (may be removed, it was for debugging purpose)

Its Launchfile call mjpeg_server for streaming and rosbridge_websocket for javascript bridge. 
This node have master as namespace.

#Special
pigpio is a special feature for raspberry pi only. It enable the use of three buttons with its gpio interface to call ROS services. It can be added in camera controler launch file. The three buttons must be interfaced on:
pin 4: take picture with device
pin 22: take network picture
pin 23: set predefined timelaps
The pins are software debounced, and react on falling edge, no need of super user to call them.

#Parameter Server
* /IP : this namespace contain a dictionnarie of device name with there IP adress as value
* /"DeviceName" : every device have its own namespace
  * file : yaml file for timelaps
  * camerasetting : camera's information
    * iso
    * aperture
    * shutterspeed
    * imageformat
    * captureSequence : dictionnary of camerasetting to take hdr pictures

**None of these are dynamic parameter**


