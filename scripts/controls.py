
PS3 = "ps3"

DEFAULT_GAMEPAD = PS3

LEFT_JOY_X = "left_joy_x"
A_BUTTON = "a_button"
B_BUTTON = "b_button"
X_BUTTON = "x_button"
Y_BUTTON = "y_button"
LEFT_BUMPER = "left_bumper"
LEFT_TRIGGER = "left_trigger"
RIGHT_TRIGGER = "right_trigger"
SELECT_BUTTON = "select_button"
START_BUTTON = "start_button"

AXES = 'a'
BUTTONS = 'b'

GAMEPAD_MAP = {
    LEFT_JOY_X : [(AXES, 0)],
    A_BUTTON : [(BUTTONS, 14)],   
    B_BUTTON : [(BUTTONS, 13)], 
    X_BUTTON : [(BUTTONS, 15)],
    Y_BUTTON : [(BUTTONS, 12)], 
    LEFT_BUMPER : [(BUTTONS, 10)],
    LEFT_TRIGGER : [(AXES, 12)],
    RIGHT_TRIGGER : [(AXES, 13)], 
    SELECT_BUTTON : [(BUTTONS, 0)], 
    START_BUTTON : [(BUTTONS, 3)]
}

gamepads = {
    PS3: 0
}


STEER = "steer"
DYNAMIC_SPEED = "dynamic"
DEAD_MANS_SWITCH = "dmg"
TOGGLE_AUTOMATIC = "auto"
TOGGLE_REVERSE = "rev"
FULL_SPEED_FORWARD = "fullAcc"
FAST_SPEED_FORWARD = "fastAcc"
SLOW_SPEED_FORWARD = "slowAcc"
FULL_SPEED_BACKWARD = "fullRev"
SLOW_SPEED_BACKWARD = "slowrev"


CONTROLS_MAP = {
    STEER: LEFT_JOY_X,
    DYNAMIC_SPEED: RIGHT_TRIGGER,
    DEAD_MANS_SWITCH: LEFT_TRIGGER,
    TOGGLE_AUTOMATIC: START_BUTTON,
    TOGGLE_REVERSE: SELECT_BUTTON,
    FULL_SPEED_FORWARD: A_BUTTON,
    FAST_SPEED_FORWARD: Y_BUTTON,
    SLOW_SPEED_FORWARD: X_BUTTON,
    FULL_SPEED_BACKWARD: LEFT_BUMPER,
    SLOW_SPEED_BACKWARD: B_BUTTON
}

def getButtons(data, controller):
    i = joysticks[controller]
    d = {}
    for key, value in GAMEPAD_MAP.iteritems():
        (t, b) = value[i]
        if t == AXES:
            d[key] = data.axes[b]
        elif t == BUTTONS:
            d[key] = data.buttons[b]
        else:
            print "GAMEPAD_MAP poorly formatted"
    return d