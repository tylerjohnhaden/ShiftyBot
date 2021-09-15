"""Autonomous Mobile Robots - Lab 1
Group: 5 "Shifty"

1. Joystick teleoperation: program and configure the joystick to operate the robot

2. Autonomous wandering with range sensor: use the IR range sensors to navigate the robot
around at constant input speed v=0.3m/s. As soon as an obstacle is detected (range < 0.2),
turn the robot toward a random different direction and keep going until it detects another
obstacle and repeat the same process.

3. Cruise control, variable speed: Use odom data and implement a PID on the speed with kp=0.1,
ki=0.001, kd=0.03. Maintain 0.3m/s then 0.5m/s and finally 0.6m/s. Implement the cruise control
behavior on top of the autonomous wandering behavior in 2 (i.e., reach and maintain a
desired speed until the robot bumps into an obstacle). Check next slide for info on the PID for
this exercise.

4. State machine: While in autonomous wandering, add joystick commands that take over and
switch between wondering and teleop behaviors. Typically: while a joystick key (one of the
buttons in the back of the controller L or R) is pressed the robot is in teleop mode and once it is
released the robot continue on its autonomous wandering behavior. Assign cruise control
speeds to the following buttons on the joystick: green button = 0.3m/s; blu = 0.5m/s; yellow =
0.6m/s; red = stop (0m/s)

5. extra: build a map using the IR sensor data and odom data. Collect data and plot the map."""

import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Shifty:
    def __init__(self):
        rospy.init_node('shifty', anonymous=True)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.map_pub = rospy.Publisher('/map', String, queue_size=4)

        self.auto = True
        self.auto_velocity_values = [0, .3, .5, .6]
        self.auto_velocity_selection = 0
        self.auto_spin_counter = 0
        self.auto_spin_direction = 1
        self.auto_spin_mean_duration = 3  # seconds
        self.auto_spin_stddev_duration = 1  # seconds

        self.dt = .1  # todo: calibrate, try .01
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)
        self.pid_vel_previous_error = 0
        self.pid_vel_integral = 0

        self.current_linear_velocity = 0
        self.current_location = [0, 0]
        self.current_direction = 0
        self.init_odom_subscriber()

        self.min_range = .2  # 0.2 meters
        self.range_scale = 1.7  # calibrated for the above value
        self.range_threshold = 5
        self.current_range_sliding_window = [0 for _ in range(10)]
        self.init_range_subscriber()

        self.cruise_control = True
        self.current_tele_linear_velocity = 0
        self.current_tele_angular_velocity = 0
        self.init_joystick_subscriber()

        # allow state variables to populate from subscribers
        rospy.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            if self.auto:
                self.auto_step()
            else:
                self.tele_step()
            self.rate.sleep()

    def auto_step(self):
        if self.auto_velocity_values[self.auto_velocity_selection] == 0:
            self.set_velocity(0, 0)
            return

        if self.currently_sees_obstacle() and self.auto_spin_counter == 0:
            self.set_velocity(0, 0)
            self.auto_spin_counter = int(random.gauss(
                self.auto_spin_mean_duration * self.hz,
                self.auto_spin_stddev_duration * self.hz
            ))
            self.auto_spin_direction = (random.randint(0, 1) * 2) - 1
            return

        if self.auto_spin_counter > 0:
            self.auto_spin_counter -= 1
            self.set_velocity(0, self.auto_spin_direction)  # todo: calibrate
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
        vel = Twist()
        vel.angular.z = target_angular_velocity

        if abs(target_linear_velocity) < 0.01:
            vel.linear.x = 0
            self.reset_pid()
        else:
            pid_output = self.pid_vel_step(target_linear_velocity, self.current_linear_velocity)
            vel.linear.x = target_linear_velocity + pid_output

        self.vel_pub.publish(vel)

    def init_odom_subscriber(self):
        def _odom_callback(data):
            self.current_linear_velocity = data.twist.twist.linear.x
            self.current_location = [data.pose.pose.position.x, data.pose.pose.position.y]

            _, _, yaw_radians = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])
            self.current_direction = yaw_radians

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self):
        def _range_callback_wrapper(left=True):
            def _left_right_callback(data):
                self.current_range_sliding_window.pop(0)
                self.current_range_sliding_window.append(data.range)

                if data.range < data.max_range / 2:
                    self.map_pub.publish('''{0};{1};{2};{3};{4}'''.format(
                        self.current_location[0],
                        self.current_location[1],
                        self.current_direction,
                        data.range,
                        'L' if left else 'R',
                    ))

            return _left_right_callback

        rospy.Subscriber("/range/fl", Range, _range_callback_wrapper(left=True))
        rospy.Subscriber("/range/fr", Range, _range_callback_wrapper(left=False))

    def init_joystick_subscriber(self):
        # ALERT! the joystick must be set to D and mode light off
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

            self.auto = all(not data.buttons[i] for i in [4, 5, 6, 7])  # (LT/RT) = teleoperation
            self.current_tele_angular_velocity = 2 * data.axes[2]  # (RJ) = turning
            self.current_tele_linear_velocity = 0.5 * data.axes[1]  # (LJ) = forward/backward

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

    def reset_pid(self):
        self.pid_vel_integral = 0
        self.pid_vel_previous_error = 0

    def currently_sees_obstacle(self):
        # error on the side of safety, calibrated for .2 meters at different speeds
        return min(self.current_range_sliding_window) < (self.min_range * self.range_scale * (
                self.auto_velocity_values[self.auto_velocity_selection] / self.auto_velocity_values[1]
        ))


if __name__ == "__main__":
    Shifty().run()
