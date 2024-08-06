#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation():
	def __Init__(self):
		self.cmd_vel = Twist()
		self.laser_msg = LaserScan()
		self.robot_stopped = False
		self.obstacle_distance = 100
		self.rospy_sub_laser = rospy.Subscriber("base_scan",LaserScan,self.laser_cb,queue_size = 1)
		self.rate = rospyRate(5)
		
	def laser_cb(self, callback):
		self.laser_msg = callback
	
	def calculate_command(self):
