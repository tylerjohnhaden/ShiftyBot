#!/usr/bin/env python
from abc import ABC, abstractmethod
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

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"


1. Go-to-goal behavior: Use odom to get the initial position of
the robot (0,0) and then send the robot to (1.5m,1.5m)
relative to its initial position. The robot has to stop when it
reaches the goal. Use odom to confirm that it moves to the
desired goal (use rqt plot to visualize data)
2. Go-to-goal behavior with constant speed in between the two
points implemented with the PID controller of Lab 1.
3. Go to multiple points: specifically, 4 corners of a square.
Implement: i) “stop, turn, and go” behavior and ii) smooth
constant linear velocity through the points.
4. Follow a path: follow a circle
5. extra: make an interesting shape (e.g, a star, a heart..) and
print the shape by using odom data"""


class Shifty(ABC):
    def __init__(self):
        rospy.init_node('shifty', anonymous=True)
        self.dt = .1  # todo: calibrate, try .01
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        # The joystick must be set to ( /D) and mode (light-off)
        self.joystick_buttons = [0 for _ in range(12)]
        self.joystick_axes = [0 for _ in range(6)]
        self.init_joystick_subscriber()

        self._global_offset_x = None
        self._global_offset_y = None
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.init_odom_subscriber()

        self.range_memory_depth = 10
        self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]
        self.init_range_subscriber()

        self.velocity_low_pass = 0.01
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # todo: join?
        self.control_loop_process = Process(target=self._control_loop)
        self.file_listener_loop_process = Process(target=self._file_listener_loop)

        # allow state variables to populate from subscribers
        rospy.sleep(0.5)

    def run(self):
        self.control_loop_process.start()
        self.file_listener_loop_process.start()

    def _control_loop(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    @abstractmethod
    def step(self):
        pass

    def _file_listener_loop(self):
        while not rospy.is_shutdown():
            for _ in range(self.hz):
                self.rate.sleep()
            self.file_step()

    def file_step(self):
        print(os.getcwd())
        # with open('command.json', 'r') as command_file:

    def init_joystick_subscriber(self):
        def _joystick_callback(data):
            self.joystick_buttons = data.buttons
            self.joystick_axes = data.axes

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            if self.global_offset_x is None or self.global_offset_y is None:
                self.global_offset_x = data.pose.pose.position.x
                self.global_offset_y = data.pose.pose.position.y

            self.linear_velocity = data.twist.twist.linear.x
            self.angular_velocity = data.twist.twist.angular.z
            self.pose_x = data.pose.pose.position.x - self._global_offset_x
            self.pose_y = data.pose.pose.position.y - self._global_offset_y
            self.pose_theta = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])[2]

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self, corners='fl,fr'):
        def _range_callback(data):
            self.range_sliding_window.pop(0)
            self.range_sliding_window.append(data.range)

        for corner in corners.split(','):
            if corner not in ['fl', 'fr', 'rl', 'rr']:
                print('Warning: triggering init_range_subscriber with incorrect corner designation')
                continue
            rospy.Subscriber(f'/range/{corner}', Range, _range_callback)

    def set_velocity(self, target_linear_velocity=0, target_angular_velocity=0):
        if abs(target_linear_velocity) < self.velocity_low_pass:
            target_linear_velocity = 0

        if abs(target_angular_velocity) < self.velocity_low_pass:
            target_angular_velocity = 0

        vel = Twist()
        vel.linear.x = target_linear_velocity
        vel.angular.z = target_angular_velocity
        self.vel_pub.publish(vel)


class ShiftyLab2Part1(Shifty):
    def __init__(self):
        super().__init__()

        self.goal_x = 1.5
        self.goal_y = 1.5
        self.goal_radius = 0.1

        self.throttle_bump = 0.075
        self.steering_Kp = 1

        self.safety_range = 0.34

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

        if distance_from_goal < self.goal_radius:
            # Robot has reached the gaol, pause run
            self.set_velocity(0, 0)
            return

        theta_d = np.arctan2(y_diff, x_diff)
        steering = self.steering_Kp * theta_d

        self.set_velocity(distance_from_goal + self.throttle_bump, steering)


if __name__ == "__main__":
    Shifty().run()
