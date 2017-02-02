#!/usr/bin/env python
# license removed for brevity
import rospy
from hw_api_ackermann.msg import AckermannDrive
from sensor_msgs.msg import Joy


pub = rospy.Publisher('steeringAngle', AckermannDrive, queue_size=10)
current_steering_angle = 50 
current_speed = 46


def callback(data):
    joystick = data.axes[0]
    throttle = data.buttons[0]
    reverse = data.buttons[1]
    samethrottle = data.buttons[3]
    ltrigger = data.axes[2]
    rtrigger = data.axes[5]
    
    
    newspeed = 46
    newangle = 50
    global current_speed
    global current_steering_angle
    if(ltrigger == -1 and rtrigger == -1):
        targetangle = -joystick*75+50
        if targetangle < current_steering_angle:
            newangle = max(current_steering_angle-2, targetangle)
        else:
            newangle = min(current_steering_angle+2, targetangle)

        if throttle == 1:
            newspeed = min(65,current_speed + 0.5)
            newspeed = max(48,newspeed)
        elif samethrottle == 1:
            newspeed = 55
        elif reverse == 1:
            newspeed = max(20,current_speed - 0.5)
            newspeed = min(28,newspeed)
        else:
            newspeed = max(46,current_speed - 0.2)
    else:
        newangle = 50
        newspeed = 46
    
    ack = AckermannDrive()
    ack.steering_angle = newangle
    ack.speed = newspeed

    current_speed = newspeed
    current_steering_angle = newangle

    print ack

    pub.publish(ack)


def converter():
    pub = rospy.Publisher('steeringAngle', AckermannDrive, queue_size=10)
    rospy.init_node('joy_converter', anonymous=False)
    print "haoeaaeoa"
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        converter()
    except rospy.ROSInterruptException:
        pass