#!/usr/bin/env python

import random

import rospy
from math import atan2

import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry

from shifty_common.ShiftyBot import ShiftyBot

from tf.transformations import euler_from_quaternion as efq


class LineBot(ShiftyBot):
    def __init__(self, name='line-bot'):
        super(LineBot, self).__init__(name)
        self.lane_pub = rospy.Publisher("bw_image", Image, queue_size=5)
        self.tape_bearing = 0
        self.bearing = 0        
        self.integral = 0
        self.prev = 0
        self.range = 420

        def _odom_Callback(odom):
	        global yaw,x,y,vx
	        quat = odom.pose.pose.orientation
	        (roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	        self.bearing = odom.twist.twist.linear.x

        def _camera_callback(data):
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            
            (rows,cols,channels)=cv_image.shape
            
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            thresh = 220
            bw_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)[1]
            kernel = np.ones((5,5),np.uint8)
            opening = cv2.morphologyEx(bw_image,cv2.MORPH_OPEN, kernel)
            
            #lower_half = bw_image[rows / 2:rows,0:cols]
            
            canny = cv2.Canny(opening, 75, 150)
            
            lines = cv2.HoughLinesP(canny, 2, np.pi / 180, 100, minLineLength=30, maxLineGap=2)
            
            if lines is not None and len(lines) > 0:
                theta_total = 0
                weights = []
                len_sumx, len_sumy = 0,0
                for line in lines:
                    x0, y0, x1, y1 = line[0]
                    w = ((x1-x0) ** 2 + (y1-y0) ** 2) ** 0.5
                    weights.append(w)
                    len_sumx += w * (x1 + x0) / 2
                    len_sumy += w * (y1 + y0) / 2
                    
                    theta_total += np.arctan2(y1 - y0, x1 - x0)
                    
                    cv2.line(canny, (x1, y1), (x0, y0), (128, 128, 128), 4)
                x_center = len_sumx / sum(weights)
                y_center = len_sumy / sum(weights)
                cv2.circle(canny, (int(x_center), int(y_center)), 3, (255,255,255), 3)  
                avg_theta = theta_total / len(lines)
            cv2.imshow('0', canny)
            cv2.waitKey(1)
                    
                #print('theta', avg_theta)
            
            # crop lower half of image
            tape_indices = []
            
            for k in range(self.range,480):   #450, 480
                horz_line = opening[k, 0:cols]
                ones = np.array([i for i in range(cols) if horz_line[i] > 0])
                if len(ones) == 0:
                    pass
                else:
                    m = np.mean(ones) - (cols / 2)
                    tape_indices.append(int(m))

            if len(tape_indices) == 0:
                self.tape_bearing = 0
            else: 
                self.tape_bearing = np.mean(tape_indices)
            
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, _camera_callback)
        rospy.Subscriber("/odom", Odometry, _odom_Callback)
        
    def pid(self, target, current):
        kp, ki, kd = 1, 0.5, 0
        dt = 0.1        
        error = target - current
        self.integral = self.integral + error * dt
        derivative = (error - self.prev)/dt
        self.prev = error
        return kp * error + ki * self.integral + kd * derivative   

    
    def step(self):
        return
        #print(self.tape_bearing)
        bearing = atan2(np.sin(-self.tape_bearing / 400),np.cos(-self.tape_bearing / 400))
        print(bearing)
        #bearing = self.pid(bearing, self.bearing)        
        #bearing = self.throttle_pid_step(bearing)
        if np.abs(bearing)>0.2:
            self.range = 420
        else:
            self.range = 460        
        if True or (np.abs(self.tape_bearing)<50):
            self.set_velocity(0, bearing)
        else:    
            self.set_velocity(0, bearing*0.5)

if __name__ == '__main__':
    shifty = LineBot()
    shifty.run()
    
