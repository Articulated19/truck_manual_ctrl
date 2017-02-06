## What is this repository for? ##
A package to translate sensor_msgs/joy to hw_api_ackermann/AckermannDrive

## How do I get set up? ##

### on truck: ###
clone the hw_api_ackermann repo and build it

fire up the roscore on the truck and run 

```
#!python

rosrun hw_api_ackermann truck.py
```


### on laptop: ###
Run 
```
#!python

export ROS_HOSTNAME=YOUR_IP
```

And 
```
#!python

export ROS_MASTER_URI=http://TRUCK_IP:11311
```

Follow this tutorial to set up the joy node http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

run 
```
#!python

rosrun joy joy_node _autorepeat_rate:=50 _coalesce_rate:=0.02
```

Clone the hw_api_ackermann repo and build it
Clone this repo and compile it on your computer

run 
```
#!python

rosrun joy_to_ackermann converter.py 
```



enjoy and drive responsibly

Controls:
for the ps3 controller:

hold left trigger to drive

steering: left joystick

throttle: right trigger

reverse: press select and then use throttle

constant speed: square=slow triangle=fast x=full speed circle=slow reverse l1=full reverse