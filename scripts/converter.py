from controls import *

STEER_RATE_CONSTANT = 22.5 #degrees per second
STEER_RATE_VARIABLE = 45 #in addition to constant rate. this is max rate

ACC_RATE_CONSTANT = 1.5 #m/s^2 
ACC_RATE_VARIABLE = 1.5 #in addition to constant rate. this is max rate

SLOW_DOWN_RATE_CONSTANT = 0.5 #m/s^2
SLOW_DOWN_RATE_VARIABLE = 0.5 #in addition to constant rate. this is max rate. for driving forward, backward is using forwards rate

class Converter:
    self.__init__(joy_rate, min_angle, max_angle, min_speed, max_speed):
        
        self.reverse_mode = False
        self.prev_rev_toggle_but = 0
        self.auto_mode = False
        self.prev_auto_toggle_but = 0

        self.current_steering_angle = 0 
        self.current_speed = 0 


        self.steer_k = (STEER_RATE_VARIABLE / float(joy_rate)) / (max_angle - min_angle)
        self.steer_m_r = -min_angle * self.steer_k
        self.steer_m_l = max_angle * self.steer_k
        self.steer_c = STEER_RATE_CONSTANT / float(joy_rate)

        self.acc_k = (ACC_RATE_CONSTANT / float(joy_rate)) / (max_speed - min_speed)
        self.acc_m_b = -min_speed * self.acc_k
        self.acc_m_f = max_speed * self.acc_k
        self.acc_c = ACC_RATE_CONSTANT / float(joy_rate) 

        self.slow_k = (SLOW_DOWN_RATE_VARIABLE / float(joy_rate)) / max_speed
        self.slow_c = SLOW_DOWN_RATE_CONSTANT / float(joy_rate)

        self.MIN_ANGLE = min_angle
        self.MAX_ANGLE = max_angle
        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed


    def getDriveCommands(buttons):

        handleReverseMode(buttons[CONTROLS_MAP[TOGGLE_REVERSE]])
        handleAutoMode(buttons[CONTROLS_MAP[TOGGLE_AUTOMATIC]])

        deadMansSwitch = self.hasDeadMansSwitch(buttons[CONTROLS_MAP[DEAD_MANS_SWITCH]])

        if self.manual && deadMansSwitch:
            newangle = self.getNewAngle(buttons[CONTROLS_MAP[STEER]])
            newspeed = self.getNewSpeed(buttons)
        else:
            newangle = 0
            newspeed = 0

        self.current_speed = newspeed
        self.current_steering_angle = newangle


        return (newspeed, newangle, deadMansSwitch, self.manual)



    def handleReverseMode(toggle_rev_but):
        if toggle_rev_but == 1:
            if self.prev_rev_toggle_but == 0:
                self.reverse_mode = not self.reverse_mode
                
        self.prev_rev_toggle_but = toggle_rev_but


    def handleAutoMode(toggle_auto_but):
        if toggle_auto_but == 1:
            if self.prev_auto_toggle_but == 0:
                self.auto_mode = not self.auto_mode

        self.prev_auto_toggle_but = toggle_auto_but

    def hasDeadMansSwitch(dms_but):
        return dms_but < 0



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

    def getTargetAngle(self, left_joy):
            if left_joy >= 0:
                return left_joy * self.MAX_ANGLE
            else:
                return -left_joy * self.MIN_ANGLE

    def leftTurnRate(self, cur_angle):
        return self.steer_c - self.steer_k * cur_angle + self.steer_m_l

    def rightTurnRate(self, cur_angle):
        return self.steer_c + self.steer_k * cur_angle + self.steer_m_r


    def getNewSpeed(buttons):
        
        targetSpeed = getTargetSpeed(buttons)

        if targetSpeed == 0:
            rate = self.getSlowDownRate(self.current_speed)
            if self.current_speed >= 0:
                newspeed = max(0,self.current_speed - rate)
            else:
                newspeed = min(0,self.current_speed + rate)
        else:
            if targetspeed < current_speed:
                rate = self.getDeAccRate(self.current_speed)
                newspeed = max(targetspeed, self.current_speed - rate)
            else:
                rate = self.getAccRate(self.current_speed)
                newspeed = min(targetspeed, self.current_speed + rate)

        return newspeed
        
    def getTargetSpeed(buttons):

        ds = buttons[CONTROLS_MAP[DYNAMIC_SPEED]]
        if ds != 1:
            if mode == 1:
                return ((2 - (ds + 1)) / 2.0) * self.MAX_SPEED
                
            else:
                return ((2 - (ds + 1)) / 2.0) * self.MIN_SPEED
            
        elif buttons[CONTROLS_MAP[FULL_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED

        elif buttons[CONTROLS_MAP[SLOW_SPEED_BACKWARD]] == 1:
            return self.MIN_SPEED * 1.0/2

        elif buttons[CONTROLS_MAP[SLOW_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED * 1.0/3

        elif buttons[CONTROLS_MAP[FAST_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED * 2.0/3

        elif buttons[CONTROLS_MAP[FULL_SPEED_BACKWARD]] == 1:
            return self.MIN_SPEED
        else:
            return 0 #no button pressed
        

    def getDeAccRate(self, cur_speed):
        return self.acc_c + self.acc_k * cur_speed + self.acc_m_b

    def getAccRate(self, cur_speed):
        return self.acc_c - self.acc_k * cur_speed + self.acc_m_f

    def getSlowDownRate(self, cur_speed):
        return self.slow_k * abs(cur_speed)