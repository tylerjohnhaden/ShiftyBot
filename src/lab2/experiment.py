#!/usr/bin/env python

import numpy as np

from shifty_common.ShiftyBot import ShiftyBot

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"
"""


def turning_radius(shifty, speed=0.4, turning_constant=.99, offset=0.5):
    shifty.state_turning = True
    shifty.set_goal(
        [[0.5, 0], [1, 0], [1, offset], [0, 0]],
        waypoint_behavior='maintain speed',
        throttle_behavior='cruise control',
        transformation='global',
        end_behavior='exit'
    )
    shifty.steering_kp = turning_constant
    shifty.cruise_velocity = speed
    shifty.run()


if __name__ == '__main__':
    shifty = ShiftyBot()

    for s in range(1, 6):
        speed = float(s) / 10
        for i in range(6):
            offset = 1 - (float(i) / 6)
            turning_radius(shifty, speed=speed, offset=offset)
