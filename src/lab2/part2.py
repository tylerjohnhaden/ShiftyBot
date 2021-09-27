#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, Range
from nav_msgs.msg import Odometry

"""Autonomous Mobile Robots - Lab 2
Group: 5 "Shifty"

1. Go-to-goal behavior with constant speed in between the two
points implemented with the PID controller of Lab 1.

Any button press on the joy will pause the program.

Any obstacle will pause the program."""


class Shifty:
    def __init__(self, goal_x=1.5, goal_y=1.5, steering_constant=1, target_velocity=0.5):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_distance = .1
        self.steering_constant = steering_constant
        self.target_velocity = target_velocity

        rospy.init_node('shifty', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.init_x = None
        self.init_y = None
        self.cur_x = 0
        self.cur_y = 0
        self.cur_linear_velocity = 0
        self.max_velocity = .6  # m/s
        self.init_odom_subscriber()

        self.min_range = .34  # todo: calibrate for close distance
        self.current_range_sliding_window = [0 for _ in range(10)]
        self.init_range_subscriber()

        # Any button press will cause the program to pause
        # The joystick must be set to D and mode light off
        self.joy_pause = False
        self.init_joystick_subscriber()

        self.dt = .1  # todo: calibrate, try .01
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)
        self.pid_vel_previous_error = 0
        self.pid_vel_integral = 0

        # allow state variables to populate from subscribers
        rospy.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            if self.joy_pause or self.currently_sees_obstacle():
                self.set_velocity(0, 0)
            else:
                self.auto_step()
            self.rate.sleep()

    def auto_step(self):
        x_diff = self.cur_x - self.init_x - self.goal_x
        y_diff = self.cur_y - self.init_y - self.goal_y
        distance_from_goal = np.sqrt((x_diff ** 2) + (y_diff ** 2))

        if distance_from_goal > self.goal_distance:
            self.set_velocity(0, 0)
            return

        theta_d = np.arctan2(y_diff, x_diff)
        steering = self.steering_constant * theta_d

        self.set_velocity(self.target_velocity, steering)

    def set_velocity(self, target_linear_velocity=.0, target_angular_velocity=.0):
        vel = Twist()
        vel.angular.z = target_angular_velocity

        pid_output = self.pid_vel_step(target_linear_velocity, self.cur_linear_velocity)
        vel.linear.x = min(self.max_velocity, target_linear_velocity + pid_output)

        self.vel_pub.publish(vel)

    def init_joystick_subscriber(self):
        # ALERT! the joystick must be set to D and mode light off
        def _joystick_callback(data):
            self.joy_pause = any(data.buttons)

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            self.cur_x = data.pose.pose.position.x
            self.cur_y = data.pose.pose.position.y
            self.cur_linear_velocity = data.twist.twist.linear.x

            if self.init_x is None or self.init_y is None:
                self.init_x = self.cur_x
                self.init_y = self.cur_y

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self):
        def _range_callback(data):
            self.current_range_sliding_window.pop(0)
            self.current_range_sliding_window.append(data.range)

        rospy.Subscriber("/range/fl", Range, _range_callback)
        rospy.Subscriber("/range/fr", Range, _range_callback)

    def currently_sees_obstacle(self):
        # error on the side of safety, calibrated for .2 meters at different speeds
        min_range = min(self.current_range_sliding_window)
        threshold = self.min_range * (self.cur_linear_velocity / 0.3)

        return min_range < threshold

    def pid_vel_step(self, set_point, measured_value, kp=.1, ki=.001, kd=.03):
        error = set_point - measured_value
        self.pid_vel_integral = self.pid_vel_integral + error * self.dt
        derivative = (error - self.pid_vel_previous_error) / self.dt
        self.pid_vel_previous_error = error
        return kp * error + ki * self.pid_vel_integral + kd * derivative

    def reset_pid(self):
        self.pid_vel_integral = 0
        self.pid_vel_previous_error = 0


if __name__ == "__main__":
    Shifty().run()
