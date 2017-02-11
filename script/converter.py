#!/usr/bin/env python
# license removed for brevity
import rospy
from hw_api_ackermann.msg import AckermannDrive
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *

"""

coalesce_interval should be 1/50 to match the autorepeat

values estimated with autorepeat_rate = 50 in mind, but not tested on the truck

angle rate should be about 0.6 degrees every call
lets say 0.3 is the constant part
lets say coefficient part varies from 0.6 to 0

speed rate should be about 0.03 every call
lets say 0.02 is constant
and coefficient part varies between 0.02 to 0

slow_down_rate should be about 0.015
lets say 0.01 is constant
and coefficient part varies between 0.01 and 0

"""


class JoyToAckermann:
    def __init__(self):
        self.current_steering_angle = 0 
        self.current_speed = 0 
        self.MAX_ANGLE = 16.0
        self.MIN_ANGLE = -21.0
        self.MAX_SPEED = 1.4
        self.MIN_SPEED = -0.97
        
        self.deadMansGrip = False
        self.mode = 1
        self.prev_select = 0
        self.prev_start = 0
        self.manual = False

        #rate is a cmd line argument later
        self.rate = rospy.get_param('joy_rate', 50.0)
        self.controller = rospy.get_param('joystick_type','ps3')

        self.STEER_RATE_CONSTANT = 22.5 #degrees per second
        self.STEER_RATE_VARIABLE = 45 #in addition to constant rate. this is max rate
        
        self.steer_k = (self.STEER_RATE_VARIABLE / float(self.rate)) / (self.MAX_ANGLE - self.MIN_ANGLE)
        self.steer_m_r = -self.MIN_ANGLE * self.steer_k
        self.steer_m_l = self.MAX_ANGLE * self.steer_k
        self.steer_c = self.STEER_RATE_CONSTANT / float(self.rate)

        self.ACC_RATE_CONSTANT = 1.5 #m/s^2 
        self.ACC_RATE_VARIABLE = 1.5 #in addition to constant rate. this is max rate

        self.acc_k = (self.ACC_RATE_CONSTANT / float(self.rate)) / (self.MAX_SPEED - self.MIN_SPEED)
        self.acc_m_b = -self.MIN_SPEED * self.acc_k
        self.acc_m_f = self.MAX_SPEED * self.acc_k
        self.acc_c = self.ACC_RATE_CONSTANT / float(self.rate) 

        self.SLOW_DOWN_RATE_CONSTANT = 0.5 #m/s^2
        self.SLOW_DOWN_RATE_VARIABLE = 0.5 #in addition to constant rate. this is max rate. for driving forward, backward is using forwards rate

        self.slow_k = (self.SLOW_DOWN_RATE_VARIABLE / float(self.rate)) / self.MAX_SPEED
        self.slow_c = self.SLOW_DOWN_RATE_CONSTANT / float(self.rate)

        

        self.ackermannPub = rospy.Publisher('man_ackermann_control', AckermannDrive, queue_size=10)
        self.manualPub = rospy.Publisher('manual_control', Bool, queue_size=10)
        self.deadMansGripPub = rospy.Publisher('dead_mans_grip', Bool, queue_size=10)

        rospy.init_node('converter', anonymous=False)
        rospy.Subscriber("joy", Joy, self.callback)
        self.last_message_time = rospy.get_time()
        

    def getTargetAngle(self, left_joy):
        if left_joy >= 0:
            return left_joy * self.MAX_ANGLE
        else:
            return -left_joy * self.MIN_ANGLE

    def leftTurnRate(self, cur_angle):
        return self.steer_c - self.steer_k * cur_angle + self.steer_m_l
    
    def rightTurnRate(self, cur_angle):
        return self.steer_c + self.steer_k * cur_angle + self.steer_m_r

    def getDeAccRate(self, cur_speed):
        return self.acc_c + self.acc_k * cur_speed + self.acc_m_b

    def getAccRate(self, cur_speed):
        return self.acc_c - self.acc_k * cur_speed + self.acc_m_f

    def getSlowDownRate(self, cur_speed):
        return self.slow_k * abs(cur_speed)
 


    def getNewAngle(self, left_joy):
        targetangle = self.getTargetAngle(left_joy)

        #steering
        if targetangle > self.current_steering_angle:
            #left
            rate = self.leftTurnRate(self.current_steering_angle)
            return min(targetangle, self.current_steering_angle + rate)
        
        else:
            #right
            rate = self.rightTurnRate(self.current_steering_angle)
            return max(targetangle, self.current_steering_angle - rate)


    def callback(self,data):
        
        
        """
        Ps3 controls:

        L2 not pressed = truck stops (can still steer)
        R2 = accelerate/reverse
        select = change mode between forward/backwards (for R2)

        Left stick = turn

        X = accelerate
        O = reverse
        triangle = like X but stops at a certain (slow) speed
        square = like triangle but higher limit

        L1 = like O but stops at a certain speed

        """
        #self.last_message_time = rospy.get_time()

        left_joy      = data.axes[0]         # 1 = left, -1 = right
        a_button      = data.buttons[14]    
        b_button      = data.buttons[13] 
        x_button      = data.buttons[15]
        y_button      = data.buttons[12]
        left_bumper   = data.buttons[10]
        left_trigger  = data.axes[12]         # -1 = pressed, 1 = not pressed
        right_trigger = data.axes[13]         # -1 = pressed, 1 = not pressed
        select_but    = data.buttons[0]        #select buton ps3 controller
        start_but     = data.buttons[3]

        print "left_joy", left_joy
        
        newangle = self.getNewAngle(left_joy)

        if select_but == 1:
            if self.prev_select == 0:
                self.mode *= -1
                
        self.prev_select = select_but
                    
        if start_but == 1:
            if self.prev_start == 0:
                self.manual = not self.manual
        
        self.prev_start = start_but

        no_gas = False

        
        if right_trigger != 1:
            if self.mode == 1:
                targetspeed = ((2 - (right_trigger+1))/2.0) * self.MAX_SPEED
                
            else:
                targetspeed = ((2 - (right_trigger+1))/2.0) * self.MIN_SPEED
            print "target ", targetspeed
        elif a_button == 1:
            targetspeed = self.MAX_SPEED

        elif b_button == 1:
            targetspeed = self.MIN_SPEED * 1.0/2

        elif x_button == 1:
            targetspeed = self.MAX_SPEED * 1.0/3

        elif y_button == 1:
            targetspeed = self.MAX_SPEED * 2.0/3

        elif left_bumper == 1:
            targetspeed = self.MIN_SPEED
        else:
            no_gas = True


        if no_gas:
            rate = self.getSlowDownRate(self.current_speed)
            if self.current_speed >= 0:
                newspeed = max(0,self.current_speed - rate)
            else:
                newspeed = min(0,self.current_speed + rate)
        else:
            if targetspeed < self.current_speed:
                rate = self.getDeAccRate(self.current_speed)
                newspeed = max(targetspeed, self.current_speed - rate)
                print "rate ", rate
            else:
                rate = self.getAccRate(self.current_speed)
                newspeed = min(targetspeed, self.current_speed + rate)
                
        if left_trigger < 0:
            self.deadMansGrip = True
        else:
            self.deadMansGrip = False

        ack = AckermannDrive()
        ack.steering_angle = newangle
        ack.speed = newspeed
        
        deadMansMessage = Bool()
        deadMansMessage.data = self.deadMansGrip

        manualMessage = Bool()
        manualMessage.data = self.manual

        self.current_speed = newspeed
        self.current_steering_angle = newangle

        print "ack ", ack
        print "mode ", self.mode

        self.ackermannPub.publish(ack)
        self.deadMansGripPub.publish(deadMansMessage)
        self.manualPub.publish(manualMessage)

if __name__ == '__main__':
    j = JoyToAckermann()
    rospy.spin()

