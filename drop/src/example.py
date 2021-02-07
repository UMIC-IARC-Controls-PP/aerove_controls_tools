#!/usr/bin/env python
from utils.offboard import child
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
child = child()
child.offboard()
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
	child.gotopose(0, 0, 5)
	r.sleep()