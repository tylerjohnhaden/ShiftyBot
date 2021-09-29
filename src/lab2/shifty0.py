#!/usr/bin/env python

from multiprocessing import Process
import random
import os

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Shifty(object):
    def __init__(self):
        # Part specific constants
        self.goals = [[1.5, 1.5]]
        self.goal_radius = 0.1
        self.throttle_bump = 0
        self.steering_Kp = .9
        self.safety_range = 0.34

        # Bot + heatbeat
        rospy.init_node('shifty', anonymous=True, log_level=rospy.DEBUG)
        self.dt = .01  # todo: calibrate, try .01
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        # The joystick must be set to ( /D) and mode (light-off)
        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]
        self.init_joystick_subscriber()

        self.is_global_pose_set = False
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.init_odom_subscriber()

        self.range_memory_depth = 100
        self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]
        self.init_range_subscriber()

        self.max_linear_velocity = 0.6
        self.max_angular_velocity = 1
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def run(self):
        rospy.loginfo('Running Control Loop ...')
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        if not self.is_global_pose_set:
            # We don't know the global coordinates yet
            self.set_velocity(0, 0)
            return

        if min(self.range_sliding_window) < self.safety_range * (self.linear_velocity / 0.3):
            # Robot is within range of obstacle, pause run
            self.set_velocity(0, 0)
            return

        if any(self.joystick_buttons):
            # Joystick button is pressed, pause run
            self.set_velocity(0, 0)
            return

        goal = self.goals[0]
        delta = goal - np.array([self.pose_x, self.pose_y])
        delta_distance = distance = np.sqrt(sum(np.square(delta)))
        delta_theta = np.arctan2(delta[1], delta[0])  # range (-pi, pi)
        control_angular_velocity = self.steering_Kp * self.fix_angle(delta_theta - self.pose_theta)

        rospy.loginfo(
            'Goal(%s)    Diff(%s)    Theta(%s)    Turn(%s)',
            delta_distance, delta, delta_theta, control_angular_velocity
        )

        if delta_distance < self.goal_radius:
            # Robot has reached the gaol, pause run
            self.set_velocity(0, 0)
            return

        # self.set_velocity(distance_from_goal + self.throttle_bump, steering)
        if False and abs(control_angular_velocity) > .1:
            self.set_velocity(0, control_angular_velocity)
        else:
            self.set_velocity(delta_distance + self.throttle_bump, control_angular_velocity)

    def init_joystick_subscriber(self):
        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            self.linear_velocity = data.twist.twist.linear.x
            self.angular_velocity = data.twist.twist.angular.z
            self.pose_x = data.pose.pose.position.x
            self.pose_y = data.pose.pose.position.y
            self.pose_theta = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])[2]

            if not self.is_global_pose_set:
                self.is_global_pose_set = True
                # translate and rotate
                rotation_matrix = np.array([
                    [np.cos(self.pose_theta), -np.sin(self.pose_theta)],
                    [np.sin(self.pose_theta), np.cos(self.pose_theta)]
                ])
                self.goals = np.array([self.pose_x, self.pose_y]) + np.matmul(rotation_matrix, self.goals)
                rospy.loginfo('Setting new goals %s', self.goals)

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self, corners='fl,fr'):
        def _range_callback(data):
            self.range_sliding_window.pop(0)
            self.range_sliding_window.append(data.range)

        for corner in corners.split(','):
            if corner not in ['fl', 'fr', 'rl', 'rr']:
                rospy.logwarn('Triggering init_range_subscriber with incorrect corner designation')
                continue
            rospy.Subscriber('/range/{0}'.format(corner), Range, _range_callback)

    def set_velocity(self, target_linear_velocity=0, target_angular_velocity=0):
        if abs(target_linear_velocity) < 0.01:
            target_linear_velocity = 0
        if abs(target_angular_velocity) < 0.001:
            target_angular_velocity = 0

        if target_linear_velocity > 0:
            target_linear_velocity = min(target_linear_velocity, self.max_linear_velocity)
        else:
            target_linear_velocity = max(target_linear_velocity, -self.max_linear_velocity)

        if target_angular_velocity > 0:
            target_angular_velocity = min(target_angular_velocity, self.max_angular_velocity)
        else:
            target_angular_velocity = max(target_angular_velocity, -self.max_angular_velocity)

        vel = Twist()
        vel.linear.x = target_linear_velocity
        vel.angular.z = target_angular_velocity
        self.vel_pub.publish(vel)

    def fix_angle(self, e):
        return np.arctan2(np.sin(e), np.cos(e))


if __name__ == "__main__":
    Shifty().run()
