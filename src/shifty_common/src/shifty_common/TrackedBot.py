"""A Tracked Bot

Publish the general odometry and current goal progress to be consumed by Mapper.
"""

import time

import rospy
from std_msgs.msg import String

from .GoalBot import GoalBot
from .RangeBot import RangeBot


class TrackedBot(RangeBot):
    tracking_headers = [
        'goal_id',
        'time',
        'goal_index',
        'goal_point',
        'x',
        'y',
        't',
        'linear_velocity',
        'angular_velocity',
        'odom_rate',
        'range_rate',
        'joystick_rate',
    ]

    def __init__(self, name='tracked-bot'):
        super(TrackedBot, self).__init__(name)
        self.tracking_pub = rospy.Publisher('tracking', String, queue_size=10)

    def post_step(self):
        self.tracking_pub.publish(';'.join(map(str, [
            self.goal_id,
            time.time(),
            self.goal_index,
            self.goal_point,
            self.pose_x,
            self.pose_y,
            self.pose_t,
            self.linear_velocity,
            self.angular_velocity,
            self.get_running_rate('odom'),
            self.get_running_rate('range'),
            self.get_running_rate('joystick'),
        ])))
