#!/usr/bin/env python

import random

import rosbag
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, Range
from nav_msgs.msg import Odometry

"""Autonomous Mobile Robots
Group: 5 "Shifty"

1. Joystick teleoperation: program and configure the joystick to operate the robot

2. Autonomous wandering with range sensor: use the IR range sensors to navigate the robot
around at constant input speed v=0.3m/s. As soon as an obstacle is detected (range < 0.2),
turn the robot toward a random different direction and keep going until it detects another
obstacle and repeat the same process. 

todo: Save a bag file with all the data and visualize the plots of the IR range sensor.

3. Cruise control, variable speed: Use odom data and implement a PID on the speed with kp=0.1,
ki=0.001, kd=0.03. Maintain 0.3m/s then 0.5m/s and finally 0.6m/s. Implement the cruise control
behavior on top of the autonomous wandering behavior in 2 (i.e., reach and maintain a
desired speed until the robot bumps into an obstacle). Check next slide for info on the PID for
this exercise. 

todo: Use rqt plot to visualize the speed of the vehicle

4. State machine: While in autonomous wandering, add joystick commands that take over and
switch between wondering and teleop behaviors. Typically: while a joystick key (one of the
buttons in the back of the controller L or R) is pressed the robot is in teleop mode and once it is
released the robot continue on its autonomous wandering behavior. Assign cruise control
speeds to the following buttons on the joystick: green button = 0.3m/s; blu = 0.5m/s; yellow =
0.6m/s; red = stop (0m/s)

todo:
5. extra: build a map using the IR sensor data and odom data. Collect data and plot the map."""

class MapPoint:


class Shifty:
    def __init__(self):
        rospy.init_node('shifty', anonymous=True)

        self.auto = True
        self.auto_velocity_values = [0, .3, .5, .6]
        self.auto_velocity_selection = 1
        self.auto_spin_counter = 0
        self.auto_spin_direction = 1

        self.dt = .1  # todo: calibrate, try .01
        self.rate = rospy.Rate(self.dt)
        self.pid_vel_previous_error = 0
        self.pid_vel_integral = 0

        # only used measure how long between heat beat log messages
        self.heart_beat_length = int(1 / self.dt)

        self.current_linear_velocity = 0
        self.current_location = [0, 0]
        self.init_odom_subscriber()

        self.min_range = .2
        self.current_range = 0
        self.init_range_subscriber()

        self.cruise_control = True
        self.current_tele_linear_velocity = 0
        self.current_tele_angular_velocity = 0
        self.init_joystick_subscriber()

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.map_pub = rospy.Publisher('map', <>, queue_size=5)

        # allow state variables to populate from subscribers
        rospy.sleep(1000)

    def run(self):
        while not rospy.is_shutdown():
            for _ in range(self.heart_beat_length):
                if self.auto:
                    self.auto_step()
                else:
                    self.tele_step()
                self.rate.sleep()
            rospy.loginfo(f'shifty: lin_vel={self.current_linear_velocity} '
                          f'ang_vel={self.current_tele_angular_velocity} '
                          f'auto={self.auto} cruise={self.cruise_control}'
                          f'range={self.current_range}')

    def auto_step(self):
        if self.auto_velocity_values[self.auto_velocity_selection] == 0:
            self.set_velocity(0, 0)
            return

        if self.current_range < self.min_range and self.auto_spin_counter == 0:
            self.auto_spin_counter = random.randint(5, 100)  # todo: calibrate
            self.auto_spin_direction = (random.randint(0, 1) * 2) - 1

        if self.auto_spin_counter > 0:
            self.auto_spin_counter -= 1
            self.set_velocity(.1, self.auto_spin_direction)  # todo: calibrate
            return

        self.set_velocity(self.auto_velocity_values[self.auto_velocity_selection], 0)

    def tele_step(self):
        if self.cruise_control:
            self.set_velocity(
                self.auto_velocity_values[self.auto_velocity_selection],
                self.current_tele_angular_velocity
            )
        else:
            self.set_velocity(
                self.current_tele_linear_velocity,
                self.current_tele_angular_velocity
            )

    def set_velocity(self, target_linear_velocity=.0, target_angular_velocity=.0):
        # todo: figure out how to use this
        # pid_output = self.pid_vel_step(target_linear_velocity, self.current_linear_velocity)

        vel = Twist()
        vel.angular.z = target_angular_velocity
        vel.linear.x = target_linear_velocity
        self.vel_pub.publish(vel)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            self.current_linear_velocity = data.twist.twist.linear.x
            self.current_location = [data.pose.pose.position.x, data.pose.pose.position.y]

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self):
        def _range_callback(data):
            self.current_range = data.range

            # todo: log map boundary points euler_from_quaternion?
            # self.map_pub.publish()

        rospy.Subscriber("/range", Range, _range_callback)

    def init_joystick_subscriber(self):
        def _joystick_callback(data):
            abxy_button_press = False
            if data.buttons[2]:  # (B) = 0 m/s
                self.auto_velocity_selection = 0
                abxy_button_press = True
            elif data.buttons[1]:  # (A) = 0.3 m/s
                self.auto_velocity_selection = 1
                abxy_button_press = True
            elif data.buttons[0]:  # (X) = 0.5 m/s
                self.auto_velocity_selection = 2
                abxy_button_press = True
            elif data.buttons[3]:  # (Y) = 0.6 m/s
                self.auto_velocity_selection = 3
                abxy_button_press = True

            self.auto = all(not data.axis[i] for i in [4, 5, 6, 7])  # (LT/RT) = teleoperation
            self.current_tele_angular_velocity = data.axis[2]  # (RJ) = turning
            self.current_tele_linear_velocity = data.axis[1]  # (LJ) = forward/backwrad

            if not self.auto and abxy_button_press:  # (LT/RT) + (A/B/X/Y) = cruise control
                self.cruise_control = True
            elif not self.auto and self.current_tele_linear_velocity:  # (LT/RT) + (LJ) = stop cruise control
                self.cruise_control = False

        rospy.Subscriber("/joy", Joy, _joystick_callback)

    def pid_vel_step(self, set_point, measured_value, kp=.1, ki=.001, kd=.03):
        error = set_point - measured_value
        self.pid_vel_integral = self.pid_vel_integral + error * self.dt
        derivative = (error - self.pid_vel_previous_error) / self.dt
        self.pid_vel_previous_error = error
        return kp * error + ki * self.pid_vel_integral + kd * derivative


if __name__ == "__main__":
    Shifty().run()