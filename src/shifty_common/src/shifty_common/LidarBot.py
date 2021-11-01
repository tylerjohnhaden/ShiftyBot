"""A Lidar Enabled Bot

"""

import rospy
from sensor_msgs.msg import LaserScan

from .VelocityBot import VelocityBot


class RangeBot(VelocityBot):
    def __init__(self, name='range-bot'):
        super(RangeBot, self).__init__(name)

        self.arc = 60
        self.sees_obstacle = False

        def _lidar_callback(data):
            self.sees_obstacle = any(data.ranges[i] < 0.5 for i in range(-(self.arc / 2), self.arc / 2))

        rospy.Subscriber('/scan', LaserScan, _lidar_callback)

