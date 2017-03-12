#string-representations of controllers
PS3 = "ps3"
XBOX = "xbox"

DEFAULT_GAMEPAD = PS3

#buttons
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

#button types
AXES = 'a'
BUTTONS = 'b'

#map button to joy message indexes
GAMEPAD_MAP = {
    LEFT_JOY_X : [(AXES, 0), (AXES, 0)],
    A_BUTTON : [(BUTTONS, 14), (BUTTONS, 0)],   
    B_BUTTON : [(BUTTONS, 13), (BUTTONS, 1)], 
    X_BUTTON : [(BUTTONS, 15), (BUTTONS, 2)],
    Y_BUTTON : [(BUTTONS, 12), (BUTTONS, 3)], 
    LEFT_BUMPER : [(BUTTONS, 10), (BUTTONS, 4)],
    LEFT_TRIGGER : [(AXES, 12), (AXES, 2)],
    RIGHT_TRIGGER : [(AXES, 13), (AXES, 5)], 
    SELECT_BUTTON : [(BUTTONS, 0), (BUTTONS, 6)], 
    START_BUTTON : [(BUTTONS, 3), (BUTTONS, 7)],
    DPAD_UP: [(BUTTONS, 4)]
}

#indexes for gamepad map
gamepads = {
    PS3: 0,
    XBOX: 1
}

# driving commands
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
JOURNEY_START = "journeystart"

# control scheme
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
    SLOW_SPEED_BACKWARD: B_BUTTON,
    JOURNEY_START: DPAD_UP
}

class GamepadMapFormatError(Exception):
     def __str__(self):
         return "The GAMEPAD_MAP is poorly formatted"


# returns dict with key = button, value = input
def getButtons(data, controller):
    i = gamepads[controller]
    d = {}
    for key, value in GAMEPAD_MAP.iteritems():
        (t, b) = value[i]
        if t == AXES:
            d[key] = data.axes[b]
        elif t == BUTTONS:
            d[key] = data.buttons[b]
        else:
            raise GamepadMapFormatError()
    if controller == XBOX:
        pass
        #d[LEFT_JOY_X] = d[LEFT_JOY_X] * -1
    return d
