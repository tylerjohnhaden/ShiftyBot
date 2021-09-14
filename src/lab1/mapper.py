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
pattern = re.compile(r'{"x":(\d+(\.\d+)?),"y":(\d+(\.\d+)?),"theta":(\d+(\.\d+)?),"d":(\d+(\.\d+)?),"side":([LR])}')
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


def update_output(out, x_pts, y_pts):
    for line in iter(out.readline, b''):
        x, y, err = parse_message(line)
        if not err:
            x_pts.append(x)
            y_pts.append(y)
    out.close()


if __name__ == '__main__':
    # Threading idea partly taken from https://stackoverflow.com/a/4896288
    p = Popen(['rostopic', 'echo', topic], stdout=PIPE, bufsize=1, close_fds=ON_POSIX)
    t = Thread(target=update_output, args=(p.stdout, x_points, y_points))
    t.daemon = True
    t.start()

    fig, ax = plt.subplots()
    ln, = plt.scatter([], [], 'ro')


    def init():
        half_length = int(room_longest_side_length / 2)
        ax.set_xlim(-half_length, half_length)
        ax.set_ylim(-half_length, half_length)
        return ln,


    def update(frame):
        ln.set_data(x_points, y_points)
        return ln,


    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=1000)
    plt.show()
