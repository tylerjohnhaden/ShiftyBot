"""A Joystick Enabled Bot

Capture buttons and axes.

NOTE: It is assumed the Logitech joystick is set to ( /D) and mode (light-off)
"""

import rospy
from sensor_msgs.msg import Joy

from .Bot import Bot


class JoystickBot(Bot):
    def __init__(self):
        super().__init__()

        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]

        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

        rospy.Subscriber("/joy", Joy, _joystick_callback)
