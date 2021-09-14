#!/usr/bin/env python

import re
import sys
from subprocess import PIPE, Popen
from threading import Thread

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty  # python 2.x
ON_POSIX = 'posix' in sys.builtin_module_names

topic = '/map'
pattern = re.compile(r'data: "([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);[LR]"')
room_longest_side_length = 20  # meters
x_points = []
y_points = []


def parse_message(m):
    match = pattern.search(m)
    if match:
        x = match.group(1)
        y = match.group(3)
        theta = match.group(5)
        d = match.group(7)
        # side = match.group(9)  # not currently used, I figured theta might be adjusted +/- based on side

        x_boundary = (np.cos(theta) * d) + x
        y_boundary = (np.sin(theta) * d) + y

        return x_boundary, y_boundary, False
    return None, None, True


def topic_listener(topic):
    popen = Popen(['rostopic', 'echo', topic], stdout=PIPE, universal_newlines=True, close_fds=ON_POSIX)
    for stdout_line in iter(popen.stdout.readline, ''):
        yield stdout_line
    popen.stdout.close()

def update_output(out):
    global x_points
    global y_points
    print('here')

    for line in iter(out.readline, b''):
        x, y, err = parse_message(line)
        if not err:
            x_points.append(x)
            y_points.append(y)
    out.close()


if __name__ == '__main__':
    for thing in topic_listener(topic=topic):
        print(thing)
        print(parse_message(thing))

    # fig, ax = plt.subplots()
    # ln, = plt.scatter([], [], 'ro')
    #
    #
    # def init():
    #     half_length = int(room_longest_side_length / 2)
    #     ax.set_xlim(-half_length, half_length)
    #     ax.set_ylim(-half_length, half_length)
    #     return ln,
    #
    #
    # def update(frame):
    #     ln.set_data(x_points, y_points)
    #     return ln,
    #
    #
    # ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=1000)
    # plt.show()
