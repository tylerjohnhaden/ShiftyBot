"""A Shifty Bot

The multiple inheritance of various "capability" nodes should allow for
diverse asynchronous state modification through subscribers.

Here should be the implementation of the step method, now that we have all state available to us.
"""

import numpy as np
import rospy

from . import JoystickBot, RangeBot, GoalBot, VelocityBot


def fix_angle(e):
    return np.arctan2(np.sin(e), np.cos(e))


class ShiftyBot(JoystickBot, RangeBot, GoalBot, VelocityBot):
    def __init__(self):
        super().__init__()

        # Control Constants
        self.throttle_pid_setpoint = 0.5
        self.throttle_bump = 0.03
        self.steering_Kp = .99
        self.throttle_Kp = .2
        self.obstacle_safety_range = 0.34

        # State Machine and Controllers
        self.throttle_pid_previous_error = 0
        self.throttle_pid_integral = 0

        self.obstacle_wait_time = 2  # seconds
        self.state_obstacle_counter = 0

        self.stop_wait_time = 1  # seconds
        self.state_stop_counter = 0

        self.state_turn_flag = True

    def set_control_constants(
            self,
            steering_Kp=.99,
            throttle_Kp=.2,
            throttle_bump=0.03,
            throttle_pid_setpoint=0.5,
            obstacle_safety_range=0.4
    ):
        self.steering_Kp = steering_Kp
        self.throttle_Kp = throttle_Kp
        self.throttle_bump = throttle_bump
        self.throttle_pid_setpoint = throttle_pid_setpoint
        self.obstacle_safety_range = obstacle_safety_range

    def step(self):
        # We don't know the global coordinates yet, wait for first odom reading
        if not self.is_global_pose_set:
            self.set_velocity(0, 0)
            return

        # Robot is within range of obstacle, pause for set time
        if self.sees_obstacle() or self.state_obstacle_counter > 0:
            self.set_velocity(0, 0)
            if self.state_obstacle_counter > 0:
                self.state_obstacle_counter -= 1
            else:
                self.state_obstacle_counter = int(self.obstacle_wait_time * self.hz)
            return

        # Joystick button is pressed, pause till button is released
        if any(self.joystick_buttons):
            self.set_velocity(0, 0)
            return

        # We recently reached a waypoint, pause for set time
        if self.state_stop_counter > 0:
            self.set_velocity(0, 0)
            self.state_stop_counter -= 1
            return

        next_waypoint = self.goal_waypoints[self.goal_point]
        delta = next_waypoint - np.array([self.pose_x, self.pose_y])
        delta_distance = np.sqrt(sum(np.square(delta)))
        control_linear_velocity = (self.throttle_Kp * delta_distance) + self.throttle_bump

        if delta_distance < self.goal_radius:
            self.set_velocity(0, 0)
            self.state_stop_counter = int(self.stop_wait_time * self.hz)
            self.state_turn_flag = True
            self.goal_index += 1
            self.goal_point = (self.goal_point + 1) % len(self.goal_waypoints)
            return

        delta_theta = np.arctan2(delta[1], delta[0])  # range (-pi, pi)
        control_angular_velocity = self.steering_Kp * fix_angle(delta_theta - self.pose_t)

        # todo: if self.state_turn_flag and abs(delta_theta) > 0.05:
        #     self.set_velocity(0, control_angular_velocity)
        # elif
        #
        #
        #
        #

        rospy.loginfo(
            'Goal(%s)    Diff(%s)    Theta(%s)    Turn(%s)',
            delta_distance, delta, delta_theta, control_angular_velocity
        )

        if delta_distance < self.goal_radius:
            # Robot has reached the gaol, pause run
            self.set_velocity(0, 0)
            return True

        # self.set_velocity(distance_from_goal + self.throttle_bump, steering)
        if False and abs(control_angular_velocity) > .1:
            self.set_velocity(0, control_angular_velocity)
        else:
            self.set_velocity(self.throttle_Kp * delta_distance + self.throttle_bump, control_angular_velocity)
        return False
