# transporter_autodock
An actionlib server, which brings the transporter to the dock! 

# prerequisites
We would need an: 
1. intel realsense d435 camera
2. sick TIM 571 lidar. 
3. My fork of find_object_2D must be installed and set up properly.

# Installation Instructions
1. go to catkin_ws/src
2. Enter the following command: **https://github.com/SynapseProgramming/transporter_autodock.git**
3. run catkin_make

# Manual

To start the autodock action server, open a new terminal and key in the following command:

**roslaunch transporter_autodock cdock_controller.launch**

To start the autodock action client, run the following command:

**rosrun transporter_autodock chargingdock_controller_client_node**
