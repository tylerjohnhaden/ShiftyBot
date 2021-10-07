#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 5. extra: make an interesting shape (e.g, a star, a heart..) and
print the shape by using odom data

This is a heart pattern!
"""

from ..nodes import ShiftyBot

radius = .3
heart_points = [
    [2/3, 0],
    [1, -1/3],
    [2/3, -2/3],
    [1/3, -2/3],
    [0, -1/3],
    [-1/3, 0],
    [0, 1/3],
    [1/3, 2/3],
    [2/3, 2/3],
    [1, 1/3],
]

for i in range(len(heart_points)):
    heart_points[i][0] *= radius
    heart_points[i][1] *= radius

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        heart_points,
        waypoint_behavior='maintain speed',
        throttle_behavior='constant pid',
        end_behavior='loop',
        reversible=True
    )
    shifty.run()
