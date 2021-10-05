#!/usr/bin/env python

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

Part 1. Go-to-goal behavior: Use odom to get the initial position of
the robot (0,0) and then send the robot to (1.5m,1.5m)
relative to its initial position. The robot has to stop when it
reaches the goal. Use odom to confirm that it moves to the
desired goal (use rqt plot to visualize data)
"""

from .nodes.ShiftyBot import ShiftyBot

if __name__ == '__main__':
    shifty = ShiftyBot()
    shifty.set_goal(
        [[1.5, 1.5]],
        waypoint_behavior='stop and turn',
        throttle_behavior='distance + bump'
    )
    shifty.run()
