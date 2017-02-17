#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
#from truck_hw_api import interpolate
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *
from converter import *
from controls import *

# Subscribes to joy messages and publish appropriate steering and speed commands,
# Based on a control scheme. Also handles safety buttons and a switch for automatic driving
class GamepadNode:
    def __init__(self):
        
        print "sleeping for 1 sec"
        rospy.sleep(1)
        
        min_angle = rospy.get_param('min_angle', -21)
        max_angle = rospy.get_param('max_angle', 16)
        min_speed = rospy.get_param('min_speed', -1)
        max_speed = rospy.get_param('max_speed', 1.4)
        gamepad_rate = rospy.get_param('gamepad/gamepad_rate', 50)

        self.converter = Converter(gamepad_rate, min_angle, max_angle, min_speed, max_speed)

        self.gamepad = rospy.get_param('gamepad/gamepad_type', DEFAULT_GAMEPAD).lower()

        if not self.gamepad in gamepads.keys():
            self.gamepad = DEFAULT_GAMEPAD

        self.manualDrivePublisher = rospy.Publisher('man_drive', AckermannDrive, queue_size=10)
        self.autoCtrlPublisher = rospy.Publisher('auto_ctrl', Bool, queue_size=10)
        self.dmsPublisher = rospy.Publisher('dead_mans_switch', Bool, queue_size=10)

        rospy.init_node('gamepad', anonymous=False)
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.loginfo("init done, subscribed to /joy and publishes to several topics")

    def callback(self,data):
        #dict with key = button, value = input
        try:
            #raises gamepad map format error
            buttons = getButtons(data, self.gamepad)

            #convert button input to driving commands, etc
            (newAngle, newSpeed, deadMansSwitch, autoCtrl) = self.converter.getDriveCommands(buttons)

            dms = Bool()
            dms.data = deadMansSwitch
            self.dmsPublisher.publish(dms)
            
            ac = Bool()
            ac.data = autoCtrl
            self.autoCtrlPublisher.publish(ac)

            #only publish if needed
            if deadMansSwitch and (not autoCtrl):
                ack = AckermannDrive()
                ack.steering_angle = newAngle
                ack.speed = newSpeed
                self.manualDrivePublisher.publish(ack)
        except GamepadMapFormatError:
            rospy.logfatal("%s, shutting down", GamepadMapFormatError.message)
            exit(0)

if __name__ == '__main__':
    j = GamepadNode()
    rospy.spin()
