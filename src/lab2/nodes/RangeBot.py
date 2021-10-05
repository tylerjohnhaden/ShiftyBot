"""A Short-Term-Memory Range Enabled Bot

Capture range measurements. Minimum of last n measurements is taken as truth.

Obstacle detection depends on current velocity, so we inherit from OdomBot.
"""

import rospy
from sensor_msgs.msg import Range

from .OdomBot import OdomBot


class RangeBot(OdomBot):
    def __init__(self):
        super().__init__()

        self.range_corners = ['fl', 'fr']
        self.range_memory_depth = 10
        self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]

        self.obstacle_safety_range = 0.34
        self.obstacle_velocity_multiplier =  3

        def _range_callback(data):
            self.range_sliding_window.pop(0)
            self.range_sliding_window.append(data.range)

        for corner in self.range_corners:
            if corner not in ['fl', 'fr', 'rl', 'rr']:
                rospy.logwarn('Triggering init_range_subscriber with incorrect corner designation')
                continue
            rospy.Subscriber('/range/{0}'.format(corner), Range, _range_callback)

    def sees_obstacle(self):
        """Calculate obstacle distance for safety

        example at max speed (0.6 m/s):
            threshold = 0.61
            sliding_window = [inf, inf, 0.8, 0.7, 0.6, inf, 0.5, inf, 0.5, inf]

            is min(sliding_window) < threshold?

            0.5 < 0.61 == Yes
        """
        threshold = self.obstacle_safety_range * self.linear_velocity * self.obstacle_velocity_multiplier
        return min(self.range_sliding_window) < threshold
