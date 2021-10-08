"""A Goal Oriented Bot

Allow setting of rich goal instructions. This "goal data object" is identifiable and static, allowing us to publish
to other nodes. This publish step occurs right at the end of a run and right at the beginning. This way any
subscribers have two opportunities to hear unique goal. It is also recommended that the goal data is published
at every waypoint.

Waypoint coordinates are typically relative, like [1.5, 1.5]. We can use the global pose to transform this into
the actual coordinates. Benefits of this is correlated values over several runs (without extra-terrestrial movement).
"""

import random

import numpy as np
import rospy
from std_msgs.msg import String

from .OdomBot import OdomBot
from .ReversibleBot import ReversibleBot
from .util import fix_angle


class GoalBot(OdomBot, ReversibleBot):
    goal_headers = [
        'goal_id',
        'goal_waypoints',
        'goal_waypoint_behavior',
        'goal_throttle_behavior',
        'goal_end_behavior',
        'goal_radius',
        'goal_transformation',
        'goal_reversible'
    ]

    default_waypoint = [[0, 0]]
    waypoint_behavior_options = ['stop and turn', 'maintain speed']
    throttle_behavior_options = ['cruise control', 'distance + bump']
    end_behavior_options = ['exit', 'loop']
    transformation_options = ['relative', 'global']

    def __init__(self, name='odom-bot'):
        super().__init__(name)

        self._goal_id_length = 8
        self.goal_id = self.generate_goal_id()
        self.goal_waypoints = np.array(self.default_waypoint)
        self.goal_radius = 0.05
        self.goal_transformation = 'relative'
        self.goal_waypoint_behavior = 'stop and turn'
        self.goal_throttle_behavior = 'cruise control'
        self.goal_end_behavior = 'exit on completion'
        self.goal_reversible = False

        self.goal_index = 0
        self.goal_point = 0

        self.goal_pub = rospy.Publisher('goal', String, queue_size=10)

    def generate_goal_id(self):
        return str(random.randint(10 ** (self._goal_id_length - 1), (10 ** self._goal_id_length) - 1))

    def transform_relative_to_global(self, goals):
        rotation_matrix = np.array([
            [np.cos(self.global_pose_t), -np.sin(self.global_pose_t)],
            [np.sin(self.global_pose_t), np.cos(self.global_pose_t)]
        ])
        return np.array([self.global_pose_x, self.global_pose_y]) + rotation_matrix.dot(goals.T).T

    def get_current_goal_delta(self):
        next_waypoint = self.goal_waypoints[self.goal_point]

        if self.goal_transformation == 'relative':
            next_waypoint = self.transform_relative_to_global(next_waypoint)

        delta = next_waypoint - np.array([self.pose_x, self.pose_y])  # (-inf, +inf) X (-inf, +inf)
        delta_theta = np.arctan2(delta[1], delta[0])  # (-pi, pi)
        delta_distance = np.sqrt(sum(np.square(delta)))  # [0, +inf)

        if not self.forward_direction:
            delta_theta = fix_angle(delta_theta + np.pi)

        return delta_theta, delta_distance

    def publish_current_goal(self):
        self.goal_pub.publish(','.join([
            self.goal_id,
            self.goal_waypoints,
            self.goal_waypoint_behavior,
            self.goal_throttle_behavior,
            self.goal_end_behavior,
            self.goal_radius,
            self.goal_transformation,
            self.goal_reversible,
        ]))

    def set_goal(
            self,
            waypoints=None,
            waypoint_behavior='stop and turn',
            throttle_behavior='cruise control',
            end_behavior='exit',
            radius=.05,
            transformation='relative',
            reversible=False,
    ):
        self.publish_current_goal()

        if waypoint_behavior not in self.waypoint_behavior_options:
            rospy.logwarn('unrecognized waypoint behavior: %s', waypoint_behavior)
            return
        if throttle_behavior not in self.throttle_behavior_options:
            rospy.logwarn('unrecognized throttle behavior: %s', throttle_behavior)
            return
        if end_behavior not in self.end_behavior_options:
            rospy.logwarn('unrecognized end behavior: %s', end_behavior)
            return
        if radius < 0.001:
            rospy.logwarn('attempting to set goal point too small: %s', radius)
            return
        if transformation not in self.transformation_options:
            rospy.logwarn('unrecognized goal transformation: %s', transformation)
            return
        if waypoints is None:
            waypoints = self.default_waypoint

        try:
            waypoints_np = np.array(waypoints)
            if waypoints_np.shape[1] != 2 or waypoints_np.size == 0:
                rospy.logwarn('attempting to set incorrect waypoints: %s', waypoints)
                return

            self.goal_waypoints = waypoints_np
            self.goal_radius = radius
            self.goal_transformation = transformation
            self.goal_waypoint_behavior = waypoint_behavior
            self.goal_throttle_behavior = throttle_behavior
            self.goal_end_behavior = end_behavior
            self.goal_id = self.generate_goal_id()
            self.goal_reversible = bool(reversible)

            self.publish_current_goal()
        except Exception as e:
            rospy.logwarn('error while setting goal: %s', e)
            return
