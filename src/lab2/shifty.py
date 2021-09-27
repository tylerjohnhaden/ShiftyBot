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

rospy.loginfo('here!')


class Shifty(object):
    def __init__(self):
        # Part specific constants
        self.goal_x = 1.5 * (3 / 2)
        self.goal_y = -1.5 * (9 / 10)
        self.goal_radius = 0.1
        self.throttle_bump = 0
        self.steering_Kp = 1.5
        self.safety_range = 0.34

        # Bot + heatbeat
        rospy.init_node('shifty', anonymous=True, log_level=rospy.DEBUG)
        self.dt = .1  # todo: calibrate, try .01
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        # The joystick must be set to ( /D) and mode (light-off)
        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]
        self.init_joystick_subscriber()

        self.is_global_pose_set = False
        self.global_offset_x = None
        self.global_offset_y = None
        self.global_offset_theta = None
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.init_odom_subscriber()

        self.range_memory_depth = 100
        self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]
        self.init_range_subscriber()

        self.velocity_low_pass = 0.001
        self.velocity_high_pass = 0.2
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # allow state variables to populate from subscribers
        rospy.sleep(0.5)
        self.file_step()

    def run(self):
        rospy.loginfo('Running Control Loop ...')
        self._control_loop()

    def _control_loop(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        if min(self.range_sliding_window) < self.safety_range * (self.linear_velocity / 0.3):
            # Robot is within range of obstacle, pause run
            self.set_velocity(0, 0)
            return

        if any(self.joystick_buttons):
            # Joystick button is pressed, pause run
            self.set_velocity(0, 0)
            return

        x_diff = self.pose_x - self.goal_x
        y_diff = self.pose_y - self.goal_y
        distance_from_goal = np.sqrt((x_diff ** 2) + (y_diff ** 2))
        rospy.loginfo('Goal = %s, xy_diff = (%s, %s)', distance_from_goal, x_diff, y_diff)

        if distance_from_goal < self.goal_radius:
            # Robot has reached the gaol, pause run
            self.set_velocity(0, 0)
            return

        theta_d = np.arctan(y_diff / x_diff)
        steering = self.steering_Kp * self.fix_angle(theta_d - self.pose_theta)

        rospy.loginfo('Robot Theta = %s, Theta_d = %s, Steering = %s', self.pose_theta, theta_d, steering)

        self.set_velocity(distance_from_goal + self.throttle_bump, steering)
        # self.set_velocity(0.1, 0)

    def _file_listener_loop(self):
        while not rospy.is_shutdown():
            for _ in range(self.hz):
                self.rate.sleep()
            self.file_step()

    def file_step(self):
        rospy.loginfo(os.getcwd())
        # with open('command.json', 'r') as command_file:

    def init_joystick_subscriber(self):
        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            if not self.is_global_pose_set:
                self.is_global_pose_set = True
                self.global_offset_x = data.pose.pose.position.x
                self.global_offset_y = data.pose.pose.position.y
                self.global_offset_theta = euler_from_quaternion([
                    data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w,
                ])[2]

                rospy.loginfo(
                    'Setting global offsets x = %s, y = %s, theta = %s',
                    self.global_offset_x,
                    self.global_offset_y,
                    self.global_offset_theta
                )

            self.linear_velocity = data.twist.twist.linear.x
            self.angular_velocity = data.twist.twist.angular.z
            self.pose_x = data.pose.pose.position.x - self.global_offset_x
            self.pose_y = data.pose.pose.position.y - self.global_offset_y
            self.pose_theta = self.fix_angle(euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])[2] - self.global_offset_theta)

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
        if abs(target_linear_velocity) < self.velocity_low_pass:
            target_linear_velocity = 0
        if abs(target_angular_velocity) < self.velocity_low_pass:
            target_angular_velocity = 0

        if target_linear_velocity > self.velocity_high_pass:
            target_linear_velocity = self.velocity_high_pass

        if target_linear_velocity < -self.velocity_high_pass:
            target_linear_velocity = -self.velocity_high_pass

        vel = Twist()
        vel.linear.x = target_linear_velocity
        vel.angular.z = target_angular_velocity
        self.vel_pub.publish(vel)

    def fix_angle(self, e):
        return np.arctan2(np.sin(e), np.cos(e))


if __name__ == "__main__":
    Shifty().run()
