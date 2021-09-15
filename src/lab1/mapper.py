#!/usr/bin/env python
import re
import sys
from subprocess import PIPE, Popen
import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

ON_POSIX = 'posix' in sys.builtin_module_names

topic = '/map'
pattern = re.compile(r'data: "([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([LR])"')
room_longest_side_length = 20  # meters todo: this is not accurate
x_points = []
y_points = []


def parse_message(m):
    try:
        match = pattern.search(m)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            theta = float(match.group(3))
            d = float(match.group(4))
            side = match.group(5)

            adjustment = 0
            if side == 'R':
                adjustment = -.1
            elif side == 'L':
                adjustment = .1

            theta += adjustment

            x_boundary = (np.cos(theta) * d) + x
            y_boundary = (np.sin(theta) * d) + y

            return x_boundary, y_boundary, False
        return None, None, True
    except Exception as e:
        print('error:', e)
        return None, None, True


def topic_listener():
    popen = Popen(['rostopic', 'echo', topic], stdout=PIPE, universal_newlines=True, close_fds=ON_POSIX)
    for stdout_line in iter(popen.stdout.readline, ''):
        yield stdout_line
    popen.stdout.close()


def background():
    global x_points
    global y_points
    for message in topic_listener():
        x, y, err = parse_message(message)
        if not err:
            x_points.append(x)
            y_points.append(y)
            print('x:', x, 'y:', y)


if __name__ == '__main__':
    background_process = threading.Thread(target=background)
    background_process.start()
    print('background rostopic listener started')

    fig, ax = plt.subplots()
    ln = plt.scatter([], [])


    def init():
        half_length = int(room_longest_side_length / 2)
        ax.set_xlim(-half_length, half_length)
        ax.set_ylim(-half_length, half_length)
        return ln,


    def update(frame):
        if len(x_points) > 0:
            ln.set_offsets(list(zip(x_points, y_points)))
        return ln,


    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=1000)
    plt.show()
