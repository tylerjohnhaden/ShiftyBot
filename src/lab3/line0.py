#!/usr/bin/env python

import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

from shifty_common.ShiftyBot import ShiftyBot
from shifty_common.util import fix_angle


def dist(x0, y0, x1, y1):
    return (((x1 - x0) ** 2) + ((y1 - y0) ** 2)) ** 0.5


def midpoint(line):
    x0, y0, x1, y1 = line
    return (x0 + x1) / 2, (y0 + y1) / 2


class LineBot(ShiftyBot):
    def __init__(self, name='line-bot'):
        super(LineBot, self).__init__(name)

        self.tape_grey_threshold = 220
        self.kernel = np.ones((5, 5), np.uint8)
        self.display = True

        self.follow_theta = 0
        self.follow_distance = 0
        self.follow_confidence = 0.5

        self.tape_memory = []

        def _camera_callback(data):
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            (rows, cols, _) = cv_image.shape

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            bw_image = cv2.threshold(gray_image, self.tape_grey_threshold, 255, cv2.THRESH_BINARY)[1]

            opening = cv2.morphologyEx(bw_image, cv2.MORPH_OPEN, self.kernel)

            # lower_half = bw_image[rows / 2:rows,0:cols]

            canny = cv2.Canny(opening, 75, 150)
            lines = cv2.HoughLinesP(canny, 2, np.pi / 180, 100, minLineLength=30, maxLineGap=2)

            tapes = self.detect_tape(lines, cols, rows)

            current_time = time.time()
            self.tape_memory = list(filter((lambda t, _: current_time - t > 0.2), self.tape_memory))
            self.tape_memory.extend((current_time, tape) for tape in tapes)

            if len(self.tape_memory) == 0:
                self.follow_theta, self.follow_distance = 0, 0
                return

            _, top_choice = max(self.tape_memory, key=(lambda t, tape: tape['confidence'] - (current_time - t)))
            self.follow_theta = top_choice['theta']
            self.follow_distance = top_choice['perpendicular_distance']
            self.follow_confidence = top_choice['confidence']

            if self.display:
                cv2.line(cv_image, (cols / 2, rows), (
                    np.cos(self.follow_theta) * self.follow_distance,
                    np.sin(self.follow_theta) * self.follow_distance
                ), 3, (255, 255, 255), 5)

            if self.display:
                cv2.imshow('0', cv_image)
                cv2.waitKey(1)

        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, _camera_callback)

    @staticmethod
    def get_follow_parameters(tape, cols, rows):
        x0, y0, x1, y1 = tape
        dx = x1 - x0
        dy = y1 - y0

        # tape_length = dist(x0, y0, x1, y1)
        tape_theta = np.arctan2(-dx, dy)
        tape_y_intercept = y1 - (tape_theta * x1)

        # i think unneeded # drop_theta = np.arctan2(dy, dx)

        drop_y_intercept = y1 - (tape_theta * x1)
        intersection_x = (drop_y_intercept - tape_y_intercept) / (np.pi / 2)
        intersection_y = (tape_theta * intersection_x) + tape_y_intercept

        drop_distance = dist(cols / 2, rows, intersection_x, intersection_y)

        return tape_theta, drop_distance

    @staticmethod
    def detect_tape(
            lines,
            cols,
            rows,
            cluster_theta_threshold=np.pi / 20,
            cluster_width_threshold=40,
            cluster_midpoint_threshold=100,
            confidence_threshold=0.1,
    ):
        if len(lines) == 0:
            return []

        clusters = [
            ([lines[0][0]], LineBot.get_follow_parameters(lines[0][0], cols, rows), midpoint(lines[0][0]))]
        for line in lines[1:]:
            for i in range(len(clusters)):
                cluster_lines, (cluster_theta, cluster_dist, cluster_midpoint) = clusters[i]
                cur_theta, cur_dist = LineBot.get_follow_parameters(line[0], cols, rows)
                cur_midpoint = midpoint(line[0])

                if abs(fix_angle(cluster_theta - cur_theta)) < cluster_theta_threshold \
                        and abs(cluster_dist - cur_dist) < cluster_width_threshold \
                        and abs(dist(*cluster_midpoint, *cur_midpoint)) < cluster_midpoint_threshold:
                    n = len(cluster_lines)
                    clusters[i][0].append(cluster_lines)
                    clusters[i][1][0] = fix_angle(((cluster_theta * n) + cur_theta) / (n + 1))
                    clusters[i][1][1] = ((cluster_dist * n) + cur_dist) / (n + 1)
                    clusters[i][2][0] = ((cluster_midpoint[0] * n) + cur_midpoint[0]) / (n + 1)
                    clusters[i][2][1] = ((cluster_midpoint[1] * n) + cur_midpoint[1]) / (n + 1)
                    break
            else:
                clusters.append(([line[0]], *LineBot.get_follow_parameters(line[0], cols, rows)))

        tapes = []
        for cluster_lines, (cluster_theta, cluster_dist, cluster_midpoint) in clusters:
            # this variable will become higher when certain parameters are met
            confidence = 1

            # this was manually calculated
            max_confidence = 20

            # the below function has a peak at 2 and treats 3+ lines slightly better than 1 line
            n = len(cluster_lines)
            confidence *= n if n < 3 else 1.25  # max 2 after this step

            # this function has a peak at 0 and treats thetas larger than pi/2 the same
            confidence *= np.exp(-(cluster_theta ** 2)) + 1  # max 4 after this step

            # this function has a peak at 0 and treats distances larger than 600 the same
            confidence *= (4 * np.exp(-((cluster_dist / 300) ** 2))) + 1  # max 20 after this step

            # todo: confidence *= some_func(avg_length_of_lines)
            # todo: confidence *= some_func(maximal_distance_between_lines)

            # normalize
            confidence /= max_confidence

            if confidence >= confidence_threshold:
                tapes.append({
                    'theta': cluster_theta,
                    'perpendicular_distance': cluster_dist,
                    'confidence': confidence
                })

        return sorted(tapes, key=(lambda t: t['confidence']), reverse=True)

    def step(self):
        K_distance_steering = 0.01
        K_theta_steering = 0.9

        steering = (-K_distance_steering * self.follow_distance) + (K_theta_steering * self.follow_theta)
        steering = min(0.3, max(-0.3, steering))

        throttle = 0.4 * self.follow_confidence

        self.set_velocity(throttle, steering)


if __name__ == '__main__':
    shifty = LineBot()
    shifty.run()

    if shifty.display:
        cv2.destroyAllWindows()
