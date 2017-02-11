#!/usr/bin/env python
import rospy
from truck_hw_api.msg import AckermannDrive
from truck_hw_api.scripts import interpolate
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *
import controls
import converter


class GamepadNode:
    def __init__(self):
        
        if (not rospy.has_param('min_angle')) or \
               (not rospy.has_param('max_angle')) or \
               (not rospy.has_param('min_speed')) or \
               (not rospy.has_param('max_speed')):
            interpolate.generateDictionaries()
            interpolate.setRosParams()
        

        min_angle = rospy.get_param('min_angle')
        max_angle = rospy.get_param('max_angle')
        min_speed = rospy.get_param('min_speed')
        max_speed = rospy.get_param('max_speed')
        joy_rate = rospy.get_param('joy_rate', 50)

        self.converter = Converter(joy_rate, min_angle, max_angle, min_speed, max_speed)


        self.joystick = rospy.get_param('joystick_type', DEFAULT_JOYSTICK).lower()

        if not self.joystick in joysticks.keys():
            self.joystick = DEFAULT_JOYSTICK

                
        self.ackermannPub = rospy.Publisher('man_drive', AckermannDrive, queue_size=10)
        self.manualPub = rospy.Publisher('auto_ctrl', Bool, queue_size=10)
        self.deadMansGripPub = rospy.Publisher('dead_mans_grip', Bool, queue_size=10)

        rospy.init_node('gamepad', anonymous=False)
        rospy.Subscriber('joy', Joy, self.callback)


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
    j = GamepadNode()
    rospy.spin()

