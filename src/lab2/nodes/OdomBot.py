"""A Odometry Enabled Bot

Capture location and velocity measurements. Capture first reading to allow relative goal calculation.

The ability to reset current pose to 0 is handled by publishing the empty command to `reset_odometry`.
"""

from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

from . import MetricBot


class OdomBot(MetricBot):
    def __init__(self, name='odom-bot'):
        super().__init__(name)

        self.is_global_pose_set = False
        self.global_pose_x = 0
        self.global_pose_y = 0
        self.global_pose_t = 0

        self.pose_x = 0
        self.pose_y = 0
        self.pose_t = 0

        self.linear_velocity = 0
        self.angular_velocity = 0

        self.reset_odom_pub = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size=10)

        def _odom_callback(data):
            self.pose_x = data.pose.pose.position.x
            self.pose_y = data.pose.pose.position.y
            self.pose_t = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])[2]

            self.linear_velocity = data.twist.twist.linear.x
            self.angular_velocity = data.twist.twist.angular.z

            if not self.is_global_pose_set:
                self.global_pose_x = self.pose_x
                self.global_pose_y = self.pose_y
                self.global_pose_t = self.pose_t
                self.is_global_pose_set = True

                rospy.loginfo(
                    'Starting Pose: \n    [X,Y] = [%s, %s] (m)\n    Theta = %s (rads)',
                    self.pose_x,
                    self.pose_y,
                    self.pose_t
                )

            self.log_event('odom')

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def reset_odom(self):
        self.reset_odom_pub.publish(Empty())
