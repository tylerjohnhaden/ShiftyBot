"""A Shifty Bot

The multiple inheritance of various "capability" nodes should allow for
diverse asynchronous state modification through subscribers.

Here should be the implementation of the step method, now that we have all state available to us.
"""

import rospy

import numpy as np

from .JoystickBot import JoystickBot
from .VelocityBot import VelocityBot
from .TrackedBot import TrackedBot
from .util import fix_angle


class ShiftyBot(JoystickBot, TrackedBot):
    def __init__(self, name='shifty-bot'):
        super(ShiftyBot, self).__init__(name)

        # Control Constants
        self.cruise_velocity = 0.5
        self.throttle_bump = 0.03
        self.steering_kp = 1.4
        self.throttle_kp = .2

        # State Machine and Controllers
        self.throttle_pid_previous_error = 0
        self.throttle_pid_integral = 0

        self.obstacle_wait_time = 2  # seconds
        self.joystick_wait_time = 0.5  # seconds
        self.stop_wait_time = 1  # seconds

        self.state_obstacle_counter = 0
        self.state_joystick_counter = 0
        self.state_stop_counter = 0
        self.state_turning = True

    def step(self):
        # We don't know the global coordinates yet, wait for first odom reading
        if not self.is_global_pose_set:
            rospy.loginfo('Waiting till Odometry data is published')
            self.set_velocity(0, 0)
            return

        # Robot is within range of obstacle, pause for set time
        if (self.sees_obstacle() or self.state_obstacle_counter > 0) and False:
            self.set_velocity(0, 0)
            if self.state_obstacle_counter > 0:
                rospy.logdebug('obstacle counter = %s', self.state_obstacle_counter)
                self.state_obstacle_counter -= 1
            else:
                rospy.loginfo('I see an obstacle in the way!')
                self.state_obstacle_counter = int(self.obstacle_wait_time * self.hz)
            return

        # Joystick button is pressed, pause half a second after button is released
        if self.is_joystick_operated():
            self.set_velocity(self.get_joystick_linear_throttle(), self.get_joystick_angular_throttle())
            rospy.loginfo(
                'Joystick operation: throttle = %s, steering = %s',
                self.get_joystick_linear_throttle(),
                self.get_joystick_angular_throttle()
            )
            self.state_joystick_counter = int(self.joystick_wait_time * self.hz)
            return

        if self.state_joystick_counter > 0:
            self.set_velocity(0, 0)
            rospy.logdebug('joystick counter = %s', self.state_obstacle_counter)
            self.state_joystick_counter -= 1
            return

        # Everything after here is related to goal, so publish relevant data
        self.publish_current_goal()

        # We recently reached a waypoint, pause for set time
        if self.state_stop_counter > 0:
            rospy.logdebug('stop counter = %s', self.state_obstacle_counter)
            self.set_velocity(0, 0)
            self.state_stop_counter -= 1
            return

        delta_t, delta_r = self.get_current_goal_delta()

        # We just reached the waypoint, handle transition to next one
        if delta_r < self.goal_radius and not self.state_turning:
            waypoint = self.goal_waypoints[self.goal_point]
            rospy.loginfo(
                'Just reached waypoint! Point %s of %s, running index = %s, XY = [%.4s %.4s]',
                self.goal_point,
                len(self.goal_waypoints),
                self.goal_index,
                waypoint[0],
                waypoint[1],
            )
            if self.goal_end_behavior == 'exit' and self.goal_point == len(self.goal_waypoints) - 1:
                self.set_velocity(0, 0)
                self.exit()
                return

            if self.goal_waypoint_behavior == 'stop and turn':
                self.set_velocity(0, 0)
                self.state_stop_counter = int(self.stop_wait_time * self.hz)
                self.state_turning = True
                rospy.loginfo('Stopping to turning towards next waypoint')

                if self.goal_reversible and abs(delta_t) > 1.1 * (np.pi / 2):
                    self.set_reverse_direction()

            self.goal_index += 1
            self.goal_point = (self.goal_point + 1) % len(self.goal_waypoints)

            if self.goal_waypoint_behavior == 'stop and turn' and self.goal_reversible:
                delta_t, _ = self.get_current_goal_delta()
                if abs(delta_t) > 1.1 * (np.pi / 2):
                    self.set_reverse_direction()
            return

        current_steering_kp = self.steering_kp
        #if delta_r < 0.2:
        #    current_steering_kp *= 2  # todo: calibrate

        angular_twist = current_steering_kp * delta_t
        if self.state_turning:
            rospy.logdebug('turning towards next waypoint: angle diff = %s', delta_t)
            self.set_velocity(0, angular_twist * 1.5)
            if abs(angular_twist) < 0.02:
                self.state_turning = False
                rospy.loginfo('Done turning towards next waypoint: angle diff = %s', delta_t)
            return

        if self.goal_throttle_behavior == 'cruise control':
            linear_twist = self.cruise_velocity + self.throttle_pid_step(self.cruise_velocity)
            # todo: detect death spiral based on delta_r delta_t and linear_twist
        else:
            linear_twist = (self.throttle_kp * delta_r) + self.throttle_bump

        if not self.forward_direction:
            linear_twist = -linear_twist

        rospy.loginfo('Driving towards next waypoint: radius diff = %s', delta_r)
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
