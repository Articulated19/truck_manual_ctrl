#!/usr/bin/env python
import rospy
from hw_api_ackermann.msg import AckermannDrive
from hw_api_ackermann.scripts import dictionaries
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *
import controls


class JoystickNode:
    def __init__(self):
        
        if (not rospy.has_param('min_angle')) or \
               (not rospy.has_param('max_angle')) or \
               (not rospy.has_param('min_speed')) or \
               (not rospy.has_param('max_speed')):
            dictionaries.generateDictionaries()
            dictionaries.setRosParams()
        

        min_angle = rospy.get_param('min_angle')
        max_angle = rospy.get_param('max_angle')
        min_speed = rospy.get_param('min_speed')
        max_speed = rospy.get_param('max_speed')
        joy_rate = rospy.get_param('joy_rate', 50)

        self.converter = Converter(joy_rate, min_angle, max_angle, min_speed, max_speed)


        self.joystick = rospy.get_param('joystick_type', DEFAULT_JOYSTICK).lower()

        if not self.joystick in joysticks.keys():
            self.joystick = DEFAULT_JOYSTICK

                
        self.ackermannPub = rospy.Publisher('man_ackermann_control', AckermannDrive, queue_size=10)
        self.manualPub = rospy.Publisher('manual_control', Bool, queue_size=10)
        self.deadMansGripPub = rospy.Publisher('dead_mans_grip', Bool, queue_size=10)

        rospy.init_node('converter', anonymous=False)
        rospy.Subscriber("joy", Joy, self.callback)


    def callback(self,data):

        buttons = getButtons(data, self.controller)

        (newAngle, newSpeed, deadMansGrip, manual) = convert.getCommands(buttons)

        deadMansMessage = Bool()
        deadMansMessage.data = deadMansGrip
        self.deadMansGripPub.publish(deadMansMessage)
        
        manualMessage = Bool()
        manualMessage.data = manual
        self.manualPub.publish(manualMessage)

        if deadMansGrip && manual:
            ack = AckermannDrive()
            ack.steering_angle = newAngle
            ack.speed = newApeed
            self.ackermannPub.publish(ack)


        

if __name__ == '__main__':
    j = JoystickNode()
    rospy.spin()

