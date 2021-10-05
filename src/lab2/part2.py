#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 2. Go-to-goal behavior with constant speed in between the two
points implemented with the PID controller of Lab 1.
"""

from .nodes.ShiftyBot import ShiftyBot

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        [[1.5, 1.5]],
        waypoint_behavior='stop and turn',
        throttle_behavior='constant pid'
    )
    shifty.run()
