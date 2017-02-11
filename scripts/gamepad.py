#!/usr/bin/env python
import rospy
from truck_hw_api.msg import AckermannDrive
from truck_hw_api.scripts import interpolate
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *
from converter import *
from controls import *


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
        gamepad_rate = rospy.get_param('gamepad_rate', 50)

        self.converter = Converter(gamepad_rate, min_angle, max_angle, min_speed, max_speed)


        self.gamepad = rospy.get_param('gamepad_type', DEFAULT_GAMEPAD).lower()

        if not self.gamepad in gamepads.keys():
            self.gamepad = DEFAULT_GAMEPAD

                
        self.manualDrivePublisher = rospy.Publisher('man_drive', AckermannDrive, queue_size=10)
        self.autoDrivePublisher = rospy.Publisher('auto_ctrl', Bool, queue_size=10)
        self.dmsPublisher = rospy.Publisher('dead_mans_switch', Bool, queue_size=10)

        rospy.init_node('gamepad', anonymous=False)
        rospy.Subscriber('joy', Joy, self.callback)


    def callback(self,data):

        buttons = getButtons(data, self.controller)

        (newAngle, newSpeed, deadMansSwitch, autoDrive) = converter.getDriveCommands(buttons)

        dms = Bool()
        dms.data = deadMansSwitch
        self.dms_publisher.publish(dms)
        
        ad = Bool()
        ad.data = autoDrive
        self.autoDrivePublisher.publish(ad)

        if deadMansSwitch && (not autoDrive):
            ack = AckermannDrive()
            ack.steering_angle = newAngle
            ack.speed = newApeed
            self.ackermannPub.publish(ack)


        

if __name__ == '__main__':
    j = GamepadNode()
    rospy.spin()

