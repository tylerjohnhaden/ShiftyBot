#!/usr/bin/env python

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

        def _odom_Callback(odom):
	        global yaw,x,y,vx
	        quat = odom.pose.pose.orientation
	        (roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	        self.bearing = odom.twist.twist.linear.x

        def _camera_callback(data):
            #rospy.loginfo('HERE! 0')
            # convert compressed image to cv2 image
            np_arr = np.fromstring(data.data,np.uint8)
            cv_image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            
            br = CvBridgeError()
            # get image dimension
            (rows,cols,channels)=cv_image.shape
            
            # convert image to black and white by thresholding
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            thresh = 240
            bw_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)[1]
            kernel = np.ones((5,5),np.uint8)
            #rows: 480 cols: 640 
            #print("rows:", rows, "   cols:", cols)
            ###METHOD II
            #(rows, cols)
            opening = cv2.morphologyEx(bw_image,cv2.MORPH_OPEN,kernel)
            #print('all', opening)
            
            # crop lower half of image
            tape_indices = []
            for k in range(460,480):   #450, 480
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
        return kp * error + ki * self.integral + kd * derivative;   
        
        
    
    def step(self):
        print(self.tape_bearing)
        bearing = atan2(np.sin(-self.tape_bearing / 400),np.cos(-self.tape_bearing / 400))
        #bearing = self.pid(bearing, self.bearing)        
        #bearing = self.throttle_pid_step(bearing)        
        if(np.abs(self.tape_bearing)<50):
            self.set_velocity(0.2, bearing*2)
        else:    
            self.set_velocity(0, bearing*0.5)

if __name__ == '__main__':
    shifty = LineBot()
    shifty.run()
    
