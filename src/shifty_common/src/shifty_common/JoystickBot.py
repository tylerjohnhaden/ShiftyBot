"""A Joystick Enabled Bot

Capture buttons and axes.

NOTE: It is assumed the Logitech joystick is set to ( /D) and mode (light-off)
"""

import rospy
from sensor_msgs.msg import Joy

from .MetricBot import MetricBot


class JoystickBot(MetricBot):
    def __init__(self, name='joystick-bot'):
        super(JoystickBot, self).__init__(name)

        self.joystick_linear_throttle_multiplier = 0.5
        self.joystick_angular_throttle_multiplier = 2

        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]

        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

            self.log_event('joystick')

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def is_joystick_operated(self):
        return any(self.joystick_buttons[i] for i in [4, 5, 6, 7])

    def get_joystick_linear_throttle(self):
        return self.joystick_linear_throttle_multiplier * self.joystick_axes[1]

    def get_joystick_angular_throttle(self):
        return self.joystick_angular_throttle_multiplier * self.joystick_axes[2]
