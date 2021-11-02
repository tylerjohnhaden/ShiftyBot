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
        
        self.out = 20

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
            lines = cv2.HoughLines(opening, 1, np.pi / 180, 200)
            
            if self.out != 0:
                self.out -= 1
            
            if self.out == 0:
                self.out = -1
            
                filename = '/home/robot5/tyler_scratch/img_' + str(random.randint(1000, 9999)) + '.png'
                cv2.imwrite(filename, opening)
            
            if lines is not None and len(lines) > 0:
                print(lines.shape)
                for r, theta in lines[0]:
                    print('r, theta', r, theta)
            
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
        pass
        #print(self.tape_bearing)
        #bearing = atan2(np.sin(-self.tape_bearing / 400),np.cos(-self.tape_bearing / 400))
        #bearing = self.pid(bearing, self.bearing)        
        #bearing = self.throttle_pid_step(bearing)        
        #if(np.abs(self.tape_bearing)<50):
        #    self.set_velocity(0.2, bearing*2)
        #else:    
        #    self.set_velocity(0, bearing*0.5)

if __name__ == '__main__':
    shifty = LineBot()
    shifty.run()
    
