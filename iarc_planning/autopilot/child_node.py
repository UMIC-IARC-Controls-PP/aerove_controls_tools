#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Int32 
from utils.offboard import child, parent

class daughter_class:
	def __init__(self):
		self.check = Int32()
		self.child_loc = Point()
		rospy.Subscriber('/check_topic', Int32, self.check_func)
		rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.loc_pose)

	def check_func(self, data):
		self.check = data
		#print(self.check)
		#print(self.check)

	def loc_pose(self, data):
		self.child_loc.x = data.pose.position.x
		self.child_loc.y = data.pose.position.y
		self.child_loc.z = data.pose.position.z


if __name__ == '__main__':
	rospy.init_node('daughter_node', anonymous=True)
	child = child()
	daughter_class = daughter_class()
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		#print(daughter_class.check)
		if (daughter_class.child_loc.x > -5):
			print('hag diya for now')
		
		else:
			print('macha diya')
			time.sleep(5)
			child.setarmc(1)
			child.offboard(1)
			print('1')
			child.gotopose(child.child_loc.x, child.child_loc.y, 10)
			print('2')
			child.gotopose(15, -10, 10)
			print('3')
			child.gotopose(10, -10, 5)
			rate = rospy.Rate(20)
			while (True):
				child.circle(0, -10, 5)
				rate.sleep()
