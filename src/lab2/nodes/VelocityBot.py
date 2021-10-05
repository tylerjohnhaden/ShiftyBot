"""A Velocity Enabled Bot

Publishing to `cmd_vel` with a Twist object sets the robot's motion. Thresholding is
important for safety, efficiency, and accuracy.
"""

from geometry_msgs.msg import Twist
import rospy

from .Bot import Bot


class VelocityBot(Bot):
    def __init__(self):
        super().__init__()
        self.lin_vel_low_pass = 0.01
        self.lin_vel_high_pass = 0.6

        self.ang_vel_low_pass = 0.001
        self.ang_vel_high_pass = 3

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.set_velocity(0, 0)

    def set_velocity(self, linear_velocity=0.0, angular_velocity=0.0):
        """Threshold target velocities and publish Twist

        The very small values of these two velocities typically result from pid and
        other controller ghost noise. We should remove this to avoid creep.

        High values of linear velocity can lead to over working the motors and errant behavior.

        Thresholding angular velocity gives more pleasant predictable curve trajectories."""

        if abs(linear_velocity) < self.lin_vel_low_pass:
            linear_velocity = 0
        if abs(angular_velocity) < self.ang_vel_low_pass:
            angular_velocity = 0

        if linear_velocity > 0:
            linear_velocity = min(linear_velocity, self.lin_vel_high_pass)
        else:
            linear_velocity = max(linear_velocity, -self.lin_vel_high_pass)

        if angular_velocity > 0:
            angular_velocity = min(angular_velocity, self.ang_vel_high_pass)
        else:
            angular_velocity = max(angular_velocity, -self.ang_vel_high_pass)

        vel = Twist()
        vel.linear.x = linear_velocity
        vel.angular.z = angular_velocity
        self.vel_pub.publish(vel)
