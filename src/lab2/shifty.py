# #!/usr/bin/env python
#
# import random
#
# import numpy as np
# import rospy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu, Joy, Range
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String, Empty
# from tf.transformations import euler_from_quaternion
#
# """
#
# It is assumed the Logitech joystick is set to ( /D) and mode (light-off)
#
# """
#
#
# def fix_angle(e):
#     return np.arctan2(np.sin(e), np.cos(e))
#
#
#
#
#
#
# class Shifty(object):
#     def __init__(self):
#         #
#         # Default Goal and Control Constants
#         #
#         self._goal_id_length = 8
#         self.goal_id = self.generate_goal_id()
#         self.goal_waypoints = np.array([[0, 0]])
#         self.goal_radius = 0.05
#         self.goal_transformation = 'relative'
#         self.goal_waypoint_behavior = 'stop and turn'
#         self.goal_throttle_behavior = 'cruise control'
#         self.goal_end_behavior = 'exit on completion'
#
#         self.throttle_pid_setpoint = 0.5
#         self.throttle_bump = 0.03
#         self.steering_Kp = .99
#         self.throttle_Kp = .2
#         self.obstacle_safety_range = 0.34
#
#         #
#         # Bot + Heartbeat
#         #
#         rospy.init_node('shifty', anonymous=True, log_level=rospy.INFO)
#         self.dt = .01  # todo: see if calibration changes behavior
#         self.hz = int(1 / self.dt)
#         self.rate = rospy.Rate(self.hz)
#
#         #
#         # Subscribers
#         #
#         self.joystick_buttons = [0 for _ in range(12)]
#         self.joystick_axes = [0 for _ in range(6)]
#         self.init_joystick_subscriber()
#
#         self.is_global_pose_set = False
#         self.global_pose_x = 0
#         self.global_pose_y = 0
#         self.global_pose_t = 0
#         self.pose_x = 0
#         self.pose_y = 0
#         self.pose_t = 0
#         self.linear_velocity = 0
#         self.angular_velocity = 0
#         self.init_odom_subscriber()
#
#         self.range_memory_depth = 100
#         self.range_sliding_window = [0 for _ in range(self.range_memory_depth)]
#         self.init_range_subscriber()
#
#         #
#         # Publishers
#         #
#         self.max_linear_velocity = 0.6
#         self.max_angular_velocity = 3
#         self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
#         self.set_velocity(0, 0)
#
#         self.goal_pub = rospy.Publisher('goal', String, queue_size=10)
#
#         self.reset_odom_pub = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size=10)
#
#         #
#         # Controllers and State Machine
#         #
#         self.goal_index = 0
#         self.goal_point = 0
#
#         self.throttle_pid_previous_error = 0
#         self.throttle_pid_integral = 0
#
#         self.obstacle_wait_time = 2  # seconds
#         self.state_obstacle_counter = 0
#
#         self.stop_wait_time = 1  # seconds
#         self.state_stop_counter = 0
#
#         self.state_turn_flag = True
#
#     def generate_goal_id(self):
#         return str(random.randint(10 ** (self._goal_id_length - 1), (10 ** self._goal_id_length) - 1))
#
#     def set_goal(
#             self,
#             waypoints=None,
#             waypoint_behavior='stop and turn',
#             throttle_behavior='cruise control',
#             end_behavior='exit',
#             radius=.05,
#             transformation='relative'
#     ):
#         if waypoint_behavior not in ['stop and turn', 'maintain speed']:
#             rospy.logwarn('unrecognized waypoint behavior: %s', waypoint_behavior)
#             return
#         if throttle_behavior not in ['constant pid', 'distance + bump']:
#             rospy.logwarn('unrecognized throttle behavior: %s', throttle_behavior)
#             return
#         if end_behavior not in ['exit', 'loop']:
#             rospy.logwarn('unrecognized throttle behavior: %s', throttle_behavior)
#             return
#         if radius < 0.001:
#             rospy.logwarn('attempting to set goal point too small: %s', radius)
#             return
#         if transformation not in ['relative', 'global']:
#             rospy.logwarn('unrecognized goal transformation: %s', transformation)
#             return
#         if waypoints is None:
#             waypoints = [[0, 0]]
#
#         try:
#             waypoints_np = np.array(waypoints)
#             if waypoints_np.shape[1] != 2 or waypoints_np.size == 0:
#                 rospy.logwarn('attempting to set incorrect waypoints: %s', waypoints)
#                 return
#
#             self.goal_waypoints = waypoints_np
#             self.goal_radius = radius
#             self.goal_transformation = transformation
#             self.goal_waypoint_behavior = waypoint_behavior
#             self.goal_throttle_behavior = throttle_behavior
#             self.goal_end_behavior = end_behavior
#             self.goal_id = self.generate_goal_id()
#         except Exception as e:
#             rospy.logwarn('error while setting goal: %s', e)
#             return
#
#     def set_control_constants(
#             self,
#             steering_Kp=.99,
#             throttle_Kp=.2,
#             throttle_bump=0.03,
#             throttle_pid_setpoint=0.5,
#             obstacle_safety_range=0.4
#     ):
#         self.steering_Kp = steering_Kp
#         self.throttle_Kp = throttle_Kp
#         self.throttle_bump = throttle_bump
#         self.throttle_pid_setpoint = throttle_pid_setpoint
#         self.obstacle_safety_range = obstacle_safety_range
#
#     def reset_odom(self):
#         self.reset_odom_pub.publish(Empty())
#         # Must reset goal id because goal points can no longer be correlated
#         self.goal_id = self.generate_goal_id()
#         self.is_global_pose_set = False
#
#     def run(self):
#         rospy.loginfo('Running Control Loop ...')
#         while not rospy.is_shutdown():
#             self.step()
#             self.rate.sleep()
#
#     def step(self):
#         # We don't know the global coordinates yet, wait for first odom reading
#         if not self.is_global_pose_set:
#             self.set_velocity(0, 0)
#             return
#
#         # Robot is within range of obstacle, pause for set time
#         if self.sees_obstacle() or self.state_obstacle_counter > 0:
#             self.set_velocity(0, 0)
#             if self.state_obstacle_counter > 0:
#                 self.state_obstacle_counter -= 1
#             else:
#                 self.state_obstacle_counter = int(self.obstacle_wait_time * self.hz)
#             return
#
#         # Joystick button is pressed, pause till button is released
#         if any(self.joystick_buttons):
#             self.set_velocity(0, 0)
#             return
#
#         # We recently reached a waypoint, pause for set time
#         if self.state_stop_counter > 0:
#             self.set_velocity(0, 0)
#             self.state_stop_counter -= 1
#             return
#
#         next_waypoint = self.goal_waypoints[self.goal_point]
#         delta = next_waypoint - np.array([self.pose_x, self.pose_y])
#         delta_distance = np.sqrt(sum(np.square(delta)))
#         control_linear_velocity = (self.throttle_Kp * delta_distance) + self.throttle_bump
#
#         if delta_distance < self.goal_radius:
#             self.set_velocity(0, 0)
#             self.state_stop_counter = int(self.stop_wait_time * self.hz)
#             self.state_turn_flag = True
#             self.goal_index += 1
#             self.goal_point = (self.goal_point + 1) % len(self.goal_waypoints)
#             return
#
#         delta_theta = np.arctan2(delta[1], delta[0])  # range (-pi, pi)
#         control_angular_velocity = self.steering_Kp * fix_angle(delta_theta - self.pose_t)
#
#         if self.state_turn_flag and abs(delta_theta) > 0.05:
#             self.set_velocity(0, control_angular_velocity)
#         elif
#
#
#
#
#
#         rospy.loginfo(
#             'Goal(%s)    Diff(%s)    Theta(%s)    Turn(%s)',
#             delta_distance, delta, delta_theta, control_angular_velocity
#         )
#
#         if delta_distance < self.goal_radius:
#             # Robot has reached the gaol, pause run
#             self.set_velocity(0, 0)
#             return True
#
#         # self.set_velocity(distance_from_goal + self.throttle_bump, steering)
#         if False and abs(control_angular_velocity) > .1:
#             self.set_velocity(0, control_angular_velocity)
#         else:
#             self.set_velocity(self.throttle_Kp * delta_distance + self.throttle_bump, control_angular_velocity)
#         return False
#
#     def init_joystick_subscriber(self):
#         def _joystick_callback(data):
#             self.joystick_buttons = data.buttons
#             self.joystick_axes = data.axes
#
#         rospy.Subscriber("/joy", Joy, _joystick_callback)
#
#     def init_odom_subscriber(self):
#         def _odom_callback(data):
#             self.linear_velocity = data.twist.twist.linear.x
#             self.angular_velocity = data.twist.twist.angular.z
#             self.pose_x = data.pose.pose.position.x
#             self.pose_y = data.pose.pose.position.y
#             self.pose_t = euler_from_quaternion([
#                 data.pose.pose.orientation.x,
#                 data.pose.pose.orientation.y,
#                 data.pose.pose.orientation.z,
#                 data.pose.pose.orientation.w,
#             ])[2]
#
#             if not self.is_global_pose_set:
#                 self.global_pose_x = self.pose_x
#                 self.global_pose_y = self.pose_y
#                 self.global_pose_t = self.pose_t
#                 self.is_global_pose_set = True
#
#                 # rospy.loginfo('Starting X,Y,T = %s, %s, %s', self.pose_x, self.pose_y, self.pose_t)
#                 # rospy.loginfo('Relative goals = %s', self.goals)
#                 # rospy.loginfo('Global rotation matrix = %s', rotation_matrix)
#
#                 # rospy.loginfo('Setting new goals %s', self.goals)
#
#         rospy.Subscriber("/odom", Odometry, _odom_callback)
#
#     def transform_relative_to_global(self, goals):
#         rotation_matrix = np.array([
#             [np.cos(self.global_pose_t), -np.sin(self.global_pose_t)],
#             [np.sin(self.global_pose_t), np.cos(self.global_pose_t)]
#         ])
#         return np.array([self.global_pose_x, self.global_pose_y]) + rotation_matrix.dot(goals.T).T
#
#     def init_range_subscriber(self, corners='fl,fr'):
#         def _range_callback(data):
#             self.range_sliding_window.pop(0)
#             self.range_sliding_window.append(data.range)
#
#         for corner in corners.split(','):
#             if corner not in ['fl', 'fr', 'rl', 'rr']:
#                 rospy.logwarn('Triggering init_range_subscriber with incorrect corner designation')
#                 continue
#             rospy.Subscriber('/range/{0}'.format(corner), Range, _range_callback)
#
#     def set_velocity(self, target_linear_velocity=0, target_angular_velocity=0):
#         if abs(target_linear_velocity) < 0.01:
#             target_linear_velocity = 0
#         if abs(target_angular_velocity) < 0.001:
#             target_angular_velocity = 0
#
#         if target_linear_velocity > 0:
#             target_linear_velocity = min(target_linear_velocity, self.max_linear_velocity)
#         else:
#             target_linear_velocity = max(target_linear_velocity, -self.max_linear_velocity)
#
#         if target_angular_velocity > 0:
#             target_angular_velocity = min(target_angular_velocity, self.max_angular_velocity)
#         else:
#             target_angular_velocity = max(target_angular_velocity, -self.max_angular_velocity)
#
#         vel = Twist()
#         vel.linear.x = target_linear_velocity
#         vel.angular.z = target_angular_velocity
#         self.vel_pub.publish(vel)
#
#     # todo: calibrate range sensor at different velocities
#     def sees_obstacle(self):
#         return min(self.range_sliding_window) < self.obstacle_safety_range * (self.linear_velocity / 0.3)
#
#
# if __name__ == "__main__":
#     shifty = Shifty()
#
#     part = 1  # pick 1 - 5
#     part3_behavior = 'i'  # pick i or ii
#
#     #
#     # Part 1 - Go to goal and stop
#     #
#     if part == 1:
#         shifty.set_goal([[1.5, 1.5]], waypoint_behavior='stop and turn', throttle_behavior='distance + bump')
#
#     #
#     # Part 2 - Go to goal (with constant speed) and stop
#     #
#     elif part == 2:
#         shifty.set_goal([[1.5, 1.5]], waypoint_behavior='stop and turn', throttle_behavior='constant pid')
#
#     #
#     # Part 3 - Go to multiple waypoints
#     #
#     elif part == 3:
#         square_width = .75
#         shifty.set_goal(
#             [
#                 [square_width / 2, square_width / 2],
#                 [square_width / 2, -square_width / 2],
#                 [-square_width / 2, -square_width / 2],
#                 [-square_width / 2, square_width / 2]
#             ],
#             waypoint_behavior='stop and turn' if part3_behavior == 'i' else 'maintain speed',
#             throttle_behavior='constant pid',
#             end_behavior='loop'
#         )
#
#     #
#     # Part 4 - Follow a circle
#     #
#     if part == 4:
#         number_of_waypoints = 10
#         radius = 0.75
#         shifty.set_goal(
#             [[
#                 np.cos(2 * np.pi * (x / number_of_waypoints)) * radius,
#                 np.sin(2 * np.pi * (x / number_of_waypoints)) * radius
#             ] for x in range(0, number_of_waypoints + 1)],
#             waypoint_behavior='maintain speed',
#             throttle_behavior='constant pid',
#             end_behavior='loop'
#         )
#
#     #
#     # Part 5 - Draw a star
#     #
#     if part == 5:
#         number_of_waypoints = 5
#         radius = 0.75
#         star_points = [[
#             np.cos(2 * np.pi * (x / number_of_waypoints)) * radius,
#             np.sin(2 * np.pi * (x / number_of_waypoints)) * radius
#         ] for x in range(0, number_of_waypoints + 1)]
#         shifty.set_goal(
#             [star_points[i] for i in [0, 2, 4, 1, 3]],
#             waypoint_behavior='stop and turn',
#             throttle_behavior='constant pid',
#             end_behavior='loop'
#         )
#
#     shifty.run()
