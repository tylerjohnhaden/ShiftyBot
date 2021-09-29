#!/usr/bin/env python

from multiprocessing import Process
import random
import os
import datetime
import json

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Shifty(object):
    def __init__(self):
        rospy.init_node('mapper', anonymous=True, log_level=rospy.DEBUG)
        self.dt = 1
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.init_odom_subscriber()

        self.range_memory_depth = 10
        self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]
        self.init_range_subscriber()

        self.id = str(random.randint(0, 9999)).ljust(3, '0')
        self.data_file_name = os.path.join(os.path.abspath(__file__), 'data_{0}.log'.format(self.id))
        self.html_file_name = os.path.join(os.path.abspath(__file__), 'graph_{0}.html'.format(self.id))

    def run(self):
        rospy.loginfo('Running Mapper Loop ...')
        i = 1
        while not rospy.is_shutdown():
            if i % 10 == 0:
                self.build_html()
            self.step()
            self.rate.sleep()
            i += 1

    def step(self):
        with open(self.data_file_name, 'a') as data_file:
            data_file.write(json.dumps({
                'datetime': str(datetime.datetime.now()),
                'x': self.pose_x,
                'y': self.pose_y,
                'theta': self.pose_theta,
                'linear_velocity': self.linear_velocity,
                'angular_velocity': self.angular_velocity,
                'walls': self.get_wall()
            }))

    def build_html(self):
        with open(self.data_file_name, 'r') as data_file:
            data = []
            for line in data_file.readlines():
                data.append(json.loads(line))
                assert 'datetime' in data[-1].keys()
                assert 'x' in data[-1].keys()
                assert 'y' in data[-1].keys()
                assert 'theta' in data[-1].keys()
                assert 'linear_velocity' in data[-1].keys()
                assert 'angular_velocity' in data[-1].keys()
                assert 'walls' in data[-1].keys()

            lowest_x = min(d['x'] for d in data)
            lowest_y = min(d['y'] for d in data)
            highest_x = max(d['x'] for d in data)
            highest_y = max(d['y'] for d in data)
            height = highest_x - lowest_x
            width = highest_y - lowest_y

            walls = []
            for d in data:
                walls.extend(d['walls'])

            with open(self.html_file_name, 'w') as html_file:
                html_file.write('''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Shifty {0}</title>
</head>
<body>
    <canvas id="graph" width="1000" height="1000"></canvas>
</body>
<script>
    var xs = {1};
    var ys = {2};
    var wxs = {5};
    var wys = {6};

    var c = document.getElementById("graph");
    var ctx = c.getContext("2d");
    ctx.moveTo(500, 500);
    for (var i = 0; i < {3}; i++) {{
        ctx.strokeStyle = 'black';
        ctx.lineTo(500 + (xs[i] * 50), 500 + (ys[i] * 50));
        ctx.stroke();
        if (i + 1 === {3}) {{
            ctx.fillStyle = 'green';
            ctx.fillRect(500 + (xs[i] * 50), 500 + (ys[i] * 50), 4, 4);
        }}
    }}

    for (var j = 0; j < {4}; j++) {{
        ctx.fillStyle = 'blue';
        ctx.fillRect(500 + (wxs[j] * 50), 500 + (wys[j] * 50), 4, 4);
    }}
    </script>
</html>'''.format(
                    self.id,
                    [d['x'] for d in data],
                    [d['y'] for d in data],
                    len(data),
                    len(walls),
                    [w[0] for w in walls],
                    [w[1] for w in walls]
                ))

    def get_wall(self):
        in_range_measurements = [m for m in self.range_sliding_window if m < 1]
        if len(in_range_measurements) == 0:
            return []

        distance = np.median(in_range_measurements)
        return [
            [(np.cos(self.pose_theta) * distance) + self.pose_x, (np.sin(self.pose_theta) * distance) + self.pose_y]
        ]

    def init_odom_subscriber(self):
        def _odom_callback(data):
            self.linear_velocity = data.twist.twist.linear.x
            self.angular_velocity = data.twist.twist.angular.z
            self.pose_x = data.pose.pose.position.x
            self.pose_y = data.pose.pose.position.y
            self.pose_theta = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ])[2]

            if not self.is_global_pose_set:
                self.is_global_pose_set = True
                # translate and rotate
                rotation_matrix = np.array([
                    [np.cos(self.pose_theta), -np.sin(self.pose_theta)],
                    [np.sin(self.pose_theta), np.cos(self.pose_theta)]
                ])
                self.goals = np.array([self.pose_x, self.pose_y]) + np.matmul(rotation_matrix, self.goals)
                rospy.loginfo('Setting new goals %s', self.goals)

        rospy.Subscriber("/odom", Odometry, _odom_callback)

    def init_range_subscriber(self, corners='fl,fr'):
        def _range_callback(data):
            self.range_sliding_window.pop(0)
            self.range_sliding_window.append(data.range)

        for corner in corners.split(','):
            if corner not in ['fl', 'fr', 'rl', 'rr']:
                rospy.logwarn('Triggering init_range_subscriber with incorrect corner designation')
                continue
            rospy.Subscriber('/range/{0}'.format(corner), Range, _range_callback)

    def set_velocity(self, target_linear_velocity=0, target_angular_velocity=0):
        if abs(target_linear_velocity) < 0.01:
            target_linear_velocity = 0
        if abs(target_angular_velocity) < 0.001:
            target_angular_velocity = 0

        if target_linear_velocity > 0:
            target_linear_velocity = min(target_linear_velocity, self.max_linear_velocity)
        else:
            target_linear_velocity = max(target_linear_velocity, -self.max_linear_velocity)

        if target_angular_velocity > 0:
            target_angular_velocity = min(target_angular_velocity, self.max_angular_velocity)
        else:
            target_angular_velocity = max(target_angular_velocity, -self.max_angular_velocity)

        vel = Twist()
        vel.linear.x = target_linear_velocity
        vel.angular.z = target_angular_velocity
        self.vel_pub.publish(vel)

    def fix_angle(self, e):
        return np.arctan2(np.sin(e), np.cos(e))


if __name__ == "__main__":
    Shifty().run()
