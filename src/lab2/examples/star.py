#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 5. extra: make an interesting shape (e.g, a star, a heart..) and
print the shape by using odom data

This is a star pattern!
"""

import numpy as np

from ..nodes import ShiftyBot

number_of_waypoints = 5
radius = 0.75
star_points = [[
    np.cos(2 * np.pi * (x / number_of_waypoints)) * radius,
    np.sin(2 * np.pi * (x / number_of_waypoints)) * radius
] for x in range(0, number_of_waypoints + 1)]

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        [star_points[i] for i in [0, 2, 4, 1, 3]],
        waypoint_behavior='stop and turn',
        throttle_behavior='constant pid',
        end_behavior='loop',
        reversible=True
    )
    shifty.run()
