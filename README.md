### What is this repository for? ###
A package to translate sensor_msgs/joy to hw_api_ackermann/AckermannDrive

### How do I get set up? ###

on truck:
clone the hw_api_ackermann repo and build it
run export ROS_HOSTNAME=TRUCK_IP
and export ROS_MASTER_URI=http://TRUCK_IP:11311
fire up the roscore on the truck and run 
rosrun hw_api_ackermann truck.py

on laptop:
Run export ROS_HOSTNAME=YOUR_IP
And export ROS_MASTER_URI=http://TRUCK_IP:11311
Follow this tutorial to set up the joy node http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
run rosrun joy joy_node _autorepeat:=50
Clone the hw_api_ackermann repo and build it
Clone this repo and compile it on your computer
run rosrun joy_to_ackermann converter.py 


enjoy and drive responsibly