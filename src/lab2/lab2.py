#!/usr/bin/env python

import numpy as np

from shifty_common.ShiftyBot import ShiftyBot

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"
"""


def part1(shifty):
    """Part 1. Go-to-goal behavior:

    Use odom to get the initial position of
    the robot (0,0) and then send the robot to (1.5m,1.5m)
    relative to its initial position. The robot has to stop when it
    reaches the goal. Use odom to confirm that it moves to the
    desired goal (use rqt plot to visualize data)
    """
    shifty.set_goal(
        [[1.5, 1.5]],
        waypoint_behavior='stop and turn',
        throttle_behavior='distance + bump'
    )


def part2(shifty):
    """Part 2. Go-to-goal behavior with constant speed in between the two
    points implemented with the PID controller of Lab 1.
    """
    shifty.set_goal(
        [[1.5, 1.5]],
        waypoint_behavior='stop and turn',
        throttle_behavior='cruise control'
    )


def part3i(shifty, square_width=1.0):
    """Part 3. Go to multiple points: specifically, 4 corners of a square.

    Implement: i) "stop, turn, and go" behavior
    """
    shifty.set_goal(
        [
            [square_width / 2.0, square_width / 2.0],
            [square_width / 2.0, -square_width / 2.0],
            [-square_width / 2.0, -square_width / 2.0],
            [-square_width / 2.0, square_width / 2.0]
        ],
        waypoint_behavior='stop and turn',
        throttle_behavior='cruise control',
        end_behavior='loop'
    )


def part3ii(shifty, square_width=1.0):
    """Part 3. Go to multiple points: specifically, 4 corners of a square.

    Implement: ii) smooth constant linear velocity through the points
    """
    shifty.set_goal(
        [
            [square_width / 2.0, square_width / 2.0],
            [square_width / 2.0, -square_width / 2.0],
            [-square_width / 2.0, -square_width / 2.0],
            [-square_width / 2.0, square_width / 2.0]
        ],
        waypoint_behavior='maintain speed',
        throttle_behavior='cruise control',
        end_behavior='loop'
    )


def part4(shifty, number_of_waypoints=10, radius=0.75):
    """Part 4. Follow a path: follow a circle
    """
    shifty.set_goal(
        [[
            np.cos(2.0 * np.pi * (float(x) / number_of_waypoints)) * radius,
            np.sin(2.0 * np.pi * (float(x) / number_of_waypoints)) * radius
        ] for x in range(0, number_of_waypoints + 1)],
        waypoint_behavior='maintain speed',
        throttle_behavior='cruise control',
        end_behavior='loop'
    )
    shifty.cruise_velocity = 0.1


def part5star(shifty, number_of_waypoints=10, radius=0.75):
    """Part 5. extra: make an interesting shape (e.g, a star, a heart..) and
    print the shape by using odom data

    This is a star pattern!
    """
    number_of_waypoints = 10
    radius = 0.75

    shifty.set_goal(
        [[
            np.cos(2 * np.pi * (x / number_of_waypoints)) * radius,
            np.sin(2 * np.pi * (x / number_of_waypoints)) * radius
        ] for x in range(0, number_of_waypoints + 1)],
        waypoint_behavior='maintain speed',
        throttle_behavior='cruise control',
        end_behavior='loop'
    )


def part5heart(shifty, number_of_waypoints=10, radius=0.75):
    """Part 5. extra: make an interesting shape (e.g, a star, a heart..) and
    print the shape by using odom data

    This is a heart pattern!
    """

    radius = .3
    heart_points = [
        [2 / 3, 0],
        [1, -1 / 3],
        [2 / 3, -2 / 3],
        [1 / 3, -2 / 3],
        [0, -1 / 3],
        [-1 / 3, 0],
        [0, 1 / 3],
        [1 / 3, 2 / 3],
        [2 / 3, 2 / 3],
        [1, 1 / 3],
    ]

    for i in range(len(heart_points)):
        heart_points[i][0] *= radius
        heart_points[i][1] *= radius

    shifty.set_goal(
        heart_points,
        waypoint_behavior='maintain speed',
        throttle_behavior='cruise control',
        end_behavior='loop',
        reversible=True
    )


if __name__ == '__main__':
    shifty = ShiftyBot()

    #part1(shifty)
    #part2(shifty)
    # part3i(shifty)
    # part3ii(shifty)  # not done
    part4(shifty)
    # part5star(shifty)
    # part5heart(shifty)

    shifty.run()
