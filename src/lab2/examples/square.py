#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 3. Go to multiple points: specifically, 4 corners of a square.
Implement: i) “stop, turn, and go” behavior and ii) smooth
constant linear velocity through the points.
"""

from ..nodes import ShiftyBot

square_width = .75
part3_behavior = 'i'

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        [
            [square_width / 2, square_width / 2],
            [square_width / 2, -square_width / 2],
            [-square_width / 2, -square_width / 2],
            [-square_width / 2, square_width / 2]
        ],
        waypoint_behavior='stop and turn' if part3_behavior == 'i' else 'maintain speed',
        throttle_behavior='constant pid',
        end_behavior='loop'
    )
    shifty.run()
