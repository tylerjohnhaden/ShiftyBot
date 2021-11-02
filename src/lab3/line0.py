#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

from shifty_common.ShiftyBot import ShiftyBot


class LineBot(ShiftyBot):
    def __init__(self, name='line-bot'):
        super(LineBot, self).__init__(name)

        self.tape_grey_threshold = 220
        self.kernel = np.ones((5, 5), np.uint8)
        self.display = True

        self.heading = 0

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

            if lines is not None and len(lines) > 0:
                sum_weight, sum_len_weight_x, sum_len_weight_y = 0, 0, 0

                for x0, y0, x1, y1 in (line[0] for line in lines):
                    w = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5

                    sum_weight += w
                    sum_len_weight_x += w * (x1 + x0) / 2
                    sum_len_weight_y += w * (y1 + y0) / 2

                    if self.display:
                        cv2.line(canny, (x1, y1), (x0, y0), (128, 128, 128), 4)

                x_center = sum_len_weight_x / sum_weight
                y_center = sum_len_weight_y / sum_weight

                dx = x_center - (cols / 2)
                dy = rows - y_center
                dtheta = np.arctan2(-dx, dy)

                self.heading = min(max(dtheta, -0.2), 0.2)

                if self.display:
                    cv2.circle(canny, (int(x_center), int(y_center)), 3, (255, 255, 255), 3)

            if self.display:
                cv2.imshow('0', canny)
                cv2.waitKey(1)

        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, _camera_callback)

    def step(self):
        self.set_velocity(0.2, self.heading * 1.5)


if __name__ == '__main__':
    shifty = LineBot()
    shifty.run()

    if shifty.display:
        cv2.destroyAllWindows()
