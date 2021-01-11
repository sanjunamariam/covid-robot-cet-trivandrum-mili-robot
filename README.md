# covid-robot-cet-trivandrum-mili-robot

MILI Robot Version 1.0 User Manual
Packages in raspberry pi (kinetic)
1. git clone the files to ROS Workspace.
2. catkin_make the workspace
3. Solve errors

Possible errors
1. Error: No package 'gstreamer-1.0' found
Solution: sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

2. issue: ERROR: cannot launch node of type [gmapping/slam_gmapping]: gmapping
sudo apt-get install ros-kinetic-gmapping

3. Error:  cannot launch node of type [map_server/map_server]: map_server
sudo apt-get install ros-kinetic-map-server
4. Error: Cannot locate node of type [soundplay_node.py] in package [sound_play]
sudo apt-get install ros-kinetic-sound-play 
5. $ cd ~/name_of_workspace/src/audio_common/sound_play/scripts
$ chmod +x say.py soundplay_node.py
$ cd ~/name_of_workspace/src/service_robot/scripts
$ chmod +x cancelgoal.py publishReachGoalSound.py clearcostmap.py reset_arduino.py CmdVel_to_Motor_arduino.py  save_pose.py diff_wheeled_robot_key fake_encoder_count.py       turtlebot3_teleop_key.py feedback_to_odom_new.py
6. ERROR: cannot launch node of type [amcl/amcl]: amcl
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-move-base

sudo apt-get install ros-kinetic-base-local-planner
sudo apt-get install ros-kinetic-dwa-local-planner


