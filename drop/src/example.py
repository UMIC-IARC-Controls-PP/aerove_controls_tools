#!/usr/bin/env python
from utils.offboard import mavcon
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
mvc = mavcon()
mvc.setarm(1)
time.sleep(2)
mvc.offboard()
mvc.gotopose(0.0,0.0,3.275)
time.sleep(20)
mvc.gotopose(1.0,0.0,3.275)
time.sleep(3)
mvc.gotopose(2.0,0.0,3.275)
time.sleep(3)
mvc.gotopose(2.0,-1.0,3.275)
time.sleep(3)
mvc.gotopose(2.0,0.0,3.275)
time.sleep(3)
mvc.gotopose(2.0,1.0,3.275)