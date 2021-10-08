"""A Velocity Enabled Bot

Publishing to `cmd_vel` with a Twist object sets the robot's motion. Thresholding is
important for safety, efficiency, and accuracy.
"""

from geometry_msgs.msg import Twist
import rospy

from .GoalBot import GoalBot


def low_pass(x, threshold):
    if abs(x) < threshold:
        return 0
    return x


def high_pass(x, threshold):
    if x > 0:
        return min(x, threshold)
    else:
        return max(x, -threshold)


class VelocityBot(GoalBot):
    def __init__(self, name='velocity-bot'):
        super().__init__(name)

        self.lin_vel_kp = 0.1
        self.lin_vel_ki = 0.001
        self.lin_vel_kd = 0.03

        self.lin_vel_pid_previous_error = 0
        self.lin_vel_pid_integral = 0

        self.lin_vel_low_pass = 0.01
        self.lin_vel_high_pass = 0.6
        self.ang_vel_low_pass = 0.001
        self.ang_vel_high_pass = 3

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.set_velocity(0, 0)

    def throttle_pid_step(self, target_linear_velocity):
        error = target_linear_velocity - self.linear_velocity
        self.lin_vel_pid_integral = self.lin_vel_pid_integral + error * self.dt
        derivative = (error - self.lin_vel_pid_previous_error) / self.dt
        self.lin_vel_pid_previous_error = error
        return self.lin_vel_kp * error + self.lin_vel_ki * self.lin_vel_pid_integral + self.lin_vel_kd * derivative

    def set_velocity(self, linear_velocity=0.0, angular_velocity=0.0):
        """Threshold target velocities and publish Twist

        The very small values of these two velocities typically result from pid and
        other controller ghost noise. We should remove this to avoid creep.

        High values of linear velocity can lead to over working the motors and errant behavior.

        Thresholding angular velocity gives more pleasant predictable curve trajectories.
        """

        vel = Twist()
        vel.linear.x = high_pass(low_pass(linear_velocity, self.lin_vel_low_pass), self.lin_vel_high_pass)
        vel.angular.z = high_pass(low_pass(angular_velocity, self.ang_vel_low_pass), self.ang_vel_high_pass)
        self.vel_pub.publish(vel)
