#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image, CompressedImage
from std_msgs.msg import Bool
from time import sleep
from random import randint
from math import atan2,sin,cos,sqrt,isinf, radians, degrees
from tf.transformations import euler_from_quaternion as efq

yaw,x,y,vx = 0,0,0,0
vel = Twist()
line_heading = 0

v_max = 0.4
v_inc = 0.01

xg = 1.9
yg = 0

obstacle = False
thresh = 0.5
arc = 120
opening = 90
min_dist = 100

def odomCallback(odom):
	global yaw,x,y,vx
	quat = odom.pose.pose.orientation
	(roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	vx = odom.twist.twist.linear.x

def scanCallback(scan):
	global obstacle, opening, min_dist
	s = [scan.ranges[i] for i in range(-arc, arc)]
	min_dist = min(s)
	min_index = s.index(min_dist) - arc
	min_index = min_index if (min_index < 360) else min_index - 720
	obstacle = any((scan.ranges[i] < thresh for i in range(-arc, arc)))
	print(obstacle, min_dist)
	prev_opening = opening
	opening = 90
	i = 0
	if(obstacle):
		while(i < 360):
			i += 1
			a = scan.ranges[min_index + i]
			b = scan.ranges[min_index - i]
			if(isinf(a) or a > thresh):
				opening = (min_index + i + 120)/2
				if(abs(prev_opening - opening) < 30):
					break
			elif(isinf(b) or b > thresh):
				opening = (min_index - i - 120)/2
				if(abs(prev_opening - opening) < 30):
					break
		#print(opening)

def camera_callback(data):
	global line_heading
	np_arr = np.fromstring(data.data, np.uint8)
	cv_image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
    
	(rows,cols,channels)=cv_image.shape
    
	gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	thresh_cam = 230
	bw_image = cv2.threshold(gray_image, thresh_cam, 255, cv2.THRESH_BINARY)[1]
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
        
		dx = x_center - (cols / 2)
		dy = rows - y_center
		dtheta = np.arctan2(-dx, dy)
        
		line_heading = min(max(dtheta,-0.2), 0.2)
		#print(dtheta)
        
		cv2.circle(canny, (int(x_center), int(y_center)), 3, (255,255,255), 3)  

	cv2.imshow('0', canny)
	cv2.waitKey(1)

error = 0
prev = 0
integral = 0
dt = 0.1
kp, ki, kd = 0.8, 0.1, 0.03
odom_reset = PoseWithCovarianceStamped()

def pid(target, current):
	global error, prev, integral, dt, kp, ki, kd
	error = target - current
	integral = integral + error * dt
	derivative = (error - prev)/dt
	prev = error
	return kp * error + ki * integral + kd * derivative;
    


def main():

	global vel, v_max, x, y, yaw, vx, xg, yg, obstacle, opening, min_dist
	rospy.loginfo("Enter main")

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	odom_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.init_node('rosbot_1', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rospy.Subscriber("/scan", LaserScan, scanCallback)
	rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, camera_callback)
	rate = rospy.Rate(10)
	i=0
	follow = True

	# Reset the odom
	while(i<20):
		i=i+1
		odom_pub.publish(odom_reset)
		rate.sleep()

	while not rospy.is_shutdown():

		theta = atan2(yg-y,xg-x)
		heading = atan2(sin(theta-yaw), cos(theta-yaw))
		vd = 0.2

		if ((abs(x-xg) < 0.3) and (abs(y-yg) < 0.3) and not follow):
			print("Goal reached!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			follow = True

		if(not follow):
			print("Distance to goal: " + str(xg-x)+ " : " + str(yg-y))
			if(obstacle):
				#print("Obstacle")
				mul = max(0.01, min_dist - thresh/2)
				opening = radians(opening)

				temp = ((heading * mul) + (opening / mul)) / (mul + 1/mul)
				heading = atan2(sin(temp), cos(temp))
				heading = min(max(3 * heading,-1), 1)
				#print(heading)
				if(degrees(abs(heading)) > 30):
					vd = 0.2 / (2 ** (abs(heading) + 0))
					print("Slow Down!, " + str(degrees(heading)) + ", " + str(vd))
		else:
			#print("Line")
			heading = line_heading * 1.5
			if(obstacle):
				vd = 0
				follow = False
				odom_pub.publish(odom_reset)
				xg,yg = 1.5,0


		#vd = vx + pid(0.3,vx)
		
		

		#vel.linear.x = min(max(vd,-v_max), v_max)
		vel.linear.x = vd
		vel.angular.z = heading
		vel_pub.publish(vel)

		#rospy.loginfo(" x: %f , y: %f , Vel: %f , Ang: %f", x, y, vel.linear.x,vel.angular.z)

		rate.sleep()
		

if __name__ == "__main__":
	main()
