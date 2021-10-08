"""A Short-Term-Memory Range Enabled Bot

Capture range measurements. Minimum of last n measurements is taken as truth.

Obstacle detection depends on current velocity, so we inherit from OdomBot.
"""

import rospy
from sensor_msgs.msg import Range

from .OdomBot import OdomBot
from .ReversibleBot import ReversibleBot


class RangeBot(OdomBot, ReversibleBot):
    def __init__(self, name='range-bot'):
        super().__init__(name)

        self.range_memory_depth = 10
        self.range_sliding_window = {d: [0 for _ in range(self.range_memory_depth)] for d in ['forward', 'backward']}

        self.obstacle_safety_range = 0.34
        self.obstacle_velocity_multiplier = 3

        def _forward_range_callback(data):
            self.range_sliding_window['forward'].pop(0)
            self.range_sliding_window['forward'].append(data.range)
            self.log_event('range-forward')

        def _backward_range_callback(data):
            self.range_sliding_window['backward'].pop(0)
            self.range_sliding_window['backward'].append(data.range)
            self.log_event('range-backward')

        rospy.Subscriber('/range/fl', Range, _forward_range_callback)
        rospy.Subscriber('/range/fr', Range, _forward_range_callback)
        rospy.Subscriber('/range/rl', Range, _backward_range_callback)
        rospy.Subscriber('/range/rr', Range, _backward_range_callback)

    def sees_obstacle(self):
        """Calculate obstacle distance for safety

        example at max speed (0.6 m/s):
            threshold = 0.61
            sliding_window = [inf, inf, 0.8, 0.7, 0.6, inf, 0.5, inf, 0.5, inf]

            is min(sliding_window) < threshold?

            0.5 < 0.61 == Yes
        """
        threshold = self.obstacle_safety_range * self.linear_velocity * self.obstacle_velocity_multiplier
        return min(self.range_sliding_window['forward' if self.forward_direction else 'backward']) < threshold
