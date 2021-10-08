"""A Shifty Bot

The multiple inheritance of various "capability" nodes should allow for
diverse asynchronous state modification through subscribers.

Here should be the implementation of the step method, now that we have all state available to us.
"""

import numpy as np

from .JoystickBot import JoystickBot
from .VelocityBot import VelocityBot
from .TrackedBot import TrackedBot
from .util import fix_angle


class ShiftyBot(JoystickBot, VelocityBot, TrackedBot):
    def __init__(self, name='shifty-bot'):
        super().__init__(name)

        # Control Constants
        self.cruise_velocity = 0.5
        self.throttle_bump = 0.03
        self.steering_kp = .99
        self.throttle_kp = .2

        # State Machine and Controllers
        self.throttle_pid_previous_error = 0
        self.throttle_pid_integral = 0

        self.obstacle_wait_time = 2  # seconds
        self.stop_wait_time = 1  # seconds

        self.state_obstacle_counter = 0
        self.state_stop_counter = 0
        self.state_turning = True

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

        delta_t, delta_r = self.get_current_goal_delta()

        # We just reached the waypoint, handle transition to next one
        if delta_r < self.goal_radius and not self.state_turning:
            if self.goal_end_behavior == 'exit' and self.goal_point == len(self.goal_waypoints) - 1:
                self.set_velocity(0, 0)
                self.exit()
                return

            if self.goal_waypoint_behavior == 'stop and turn':
                self.set_velocity(0, 0)
                self.state_stop_counter = int(self.stop_wait_time * self.hz)
                self.state_turning = True

                if abs(delta_t) > (np.pi / 2):
                    self.set_reverse_direction()

            self.goal_index += 1
            self.goal_point = (self.goal_point + 1) % len(self.goal_waypoints)
            self.publish_current_goal()
            return

        angular_twist = self.steering_kp * fix_angle(delta_t - self.pose_t)
        if self.state_turning:
            self.set_velocity(0, angular_twist)
            return

        if self.goal_throttle_behavior == 'cruise control':
            linear_twist = self.cruise_velocity + self.throttle_pid_step(self.cruise_velocity)
        else:
            linear_twist = (self.throttle_kp * delta_r) + self.throttle_bump

        self.set_velocity(linear_twist, angular_twist)

    def set_control_constants(
            self,
            steering_kp,
            throttle_kp,
            throttle_bump,
            cruise_velocity,
            obstacle_safety_range
    ):
        self.steering_kp = steering_kp
        self.throttle_kp = throttle_kp
        self.throttle_bump = throttle_bump
        self.cruise_velocity = cruise_velocity
        self.obstacle_safety_range = obstacle_safety_range
