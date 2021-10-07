"""A Joystick Enabled Bot

Capture buttons and axes.

NOTE: It is assumed the Logitech joystick is set to ( /D) and mode (light-off)
"""

import rospy
from sensor_msgs.msg import Joy

from . import MetricBot


class JoystickBot(MetricBot):
    def __init__(self, name='joystick-bot'):
        super().__init__(name)

        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]

        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

            self.log_event('joystick')

        rospy.Subscriber("/joy", Joy, _joystick_callback)
