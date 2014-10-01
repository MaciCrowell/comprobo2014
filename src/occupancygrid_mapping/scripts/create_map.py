#!/usr/bin/env python
## Simple mapping algorithm based on occupancy grids

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi, sqrt, ceil
from matplotlib.pyplot import imshow
import cv2
import numpy as np

class OccupancyGridMapper:
	def __init__(self):
		cv2.namedWindow("map")
		rospy.init_node("occupancy_grid_mapper")
		self.origin = [-10, -10]
		self.resolution = .05
		self.n = 400
		self.log_odds = np.zeros((self.n,self.n))
		rospy.Subscriber("scan", LaserScan, self.process_scan)
		self.tf_listener = TransformListener()

	def process_scan(self, msg):
		# get pose according to the odometry
		p = PoseStamped(header=Header(stamp=msg.header.stamp - rospy.Duration(0.2),frame_id="base_link"), pose=Pose())
		self.odom_pose = self.tf_listener.transformPose("odom", p)
		# convert the odom pose to the tuple (x,y,theta)
		self.odom_pose = OccupancyGridMapper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
		for i in range(360):
			if msg.ranges[i] > 0.0 and msg.ranges[i] < 5.0:
				# draw a picture for this!!
				map_x = self.odom_pose[0] + msg.ranges[i]*cos(i*pi/180.0+self.odom_pose[2])
				map_y = self.odom_pose[1] + msg.ranges[i]*sin(i*pi/180.0+self.odom_pose[2])

				x_detect = int((map_x - self.origin[0])/self.resolution)
				y_detect = int((map_y - self.origin[1])/self.resolution)

				u = (map_x - self.odom_pose[0], map_y - self.odom_pose[1])
				magnitude = sqrt(u[0]**2 + u[1]**2)
				n_steps = int(ceil(magnitude/self.resolution))
				u_step = (u[0]/(n_steps-1), u[1]/(n_steps-1))
				marked = set()
				for i in range(n_steps):
					curr_x = self.odom_pose[0] + i*u_step[0]
					curr_y = self.odom_pose[1] + i*u_step[1]
					if (curr_x < self.origin[0] or
						curr_x > self.origin[0] + self.n*self.resolution or
						curr_y < self.origin[1] or
						curr_y > self.origin[1] + self.n*self.resolution):
						break
					x_ind = int((curr_x - self.origin[0])/self.resolution)
					y_ind = int((curr_y - self.origin[1])/self.resolution)
					if x_ind == x_detect and y_ind == y_detect:
						break
					if not((x_ind,y_ind) in marked):
						self.log_odds[x_ind,y_ind] += 1.0
					else:
						marked.add((x_ind,y_ind))
				self.log_odds[x_detect, y_detect] -= 1.0

		# create the image from the log odds
		im = np.zeros(self.log_odds.shape)
		for i in range(im.shape[0]):
			for j in range(im.shape[1]):
				if self.log_odds[i,j] > 2:
					im[i,j] = 1.0
				elif self.log_odds[i,j] < -2:
					im[i,j] = 0.0
				else:
					im[i,j] = 0.5 
		cv2.imshow("map",im)
		cv2.waitKey(10)

	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			r.sleep()

if __name__ == '__main__':
    try:
        node = OccupancyGridMapper()
        node.run()
    except rospy.ROSInterruptException: pass
