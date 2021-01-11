#CET Robotics and Automation 2018 Batch
TO RUN SERVICE ROBOT
$ roslaunch service_robot simulation_navigation.launch

TO RUN HARDWARE MAPPING
$ roslaunch service_robot hardware_mapping_with_joystick.launch 











ROS BRIDGE WEBSOCKET
----------------------------------------------
#INSTALL rosbridge_suite from Ros wiki

http://wiki.ros.org/rosbridge_suite

#Clone roslibjs and install dependencies

$ git clone https://github.com/RobotWebTools/roslibjs.git
$ sudo apt install npm
$ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

#Launching rosbridge websocket

$ roslaunch rosbridge_server rosbridge_websocket.launch



