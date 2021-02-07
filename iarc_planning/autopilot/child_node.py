#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32 
from utils.offboard import child, parent

class daughter_class:
	def __init__(self):
		self.check = Float32()
		self.child_loc = Point()
		rospy.Subscriber('/uav1/mavros/state', State, self.check_func)
		rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.loc_pose)

	def check_func(self, data):
		self.check = data.mode
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
		if (daughter_class.check != 'OFFBOARD'):
			print('hag diya for now')
		
		else:
			print('macha diya')
			child.setarmc(1)
			child.offboard(1)
			child.gotopose(child.child_loc.x, child.child_loc.y, child.child_loc.z + 5)
			child.FINDER(-10, 0, 5)
		r.sleep()
