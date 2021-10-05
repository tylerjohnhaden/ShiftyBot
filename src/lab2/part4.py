#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 4. Follow a path: follow a circle
"""

import numpy as np

from .nodes.ShiftyBot import ShiftyBot

number_of_waypoints = 10
radius = 0.75

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        [[
            np.cos(2 * np.pi * (x / number_of_waypoints)) * radius,
            np.sin(2 * np.pi * (x / number_of_waypoints)) * radius
        ] for x in range(0, number_of_waypoints + 1)],
        waypoint_behavior='maintain speed',
        throttle_behavior='constant pid',
        end_behavior='loop'
    )
    shifty.run()
