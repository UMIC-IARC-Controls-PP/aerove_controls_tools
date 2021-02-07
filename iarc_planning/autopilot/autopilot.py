#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool
import multiprocessing 
from threading import Thread
import thread
from utils.offboard import child, parent
from std_msgs.msg import Bool



class autopilot:
	def __init__(self):
		self.child = child()
		self.parent = parent()
		self.currentstate = 'STANDBY'
		self.isMastDetected = False
		self.isModuleDetected = False
		self.moduleLocation = Point()
		self.mastLocation = Point()
		self.mastLocation.x = 10
		self.mastLocation.y = 0
		self.mastLocation.z = 10
		# MAKE A SUBSCRIBER FOR THE VELOCITY VARIABLE

	def TAKEOFF(self, z):
		self.parent.setarmp(1)
		self.parent.offboard(1)
		self.parent.gotopose(0, 0, z)

	def LAP(self):
		wplist = []*8
		for wp in wplist:
			dist= np.sqrt(((self.parent.parent_loc.x-wp[0])**2) + ((self.parent.parent_loc.y-wp[1])**2) + ((self.parent.parent_loc.z-wp[2])**2))
   			rate = rospy.Rate(20)
	    	while(dist>0.5):
		        self.parent.og_gotopose(wp[0],wp[1],wp[2],self.var)
		        dist= np.sqrt(((mvc.pt.x-point[0])**2) + ((mvc.pt.y-point[1])**2) + ((mvc.pt.z-point[2])**2))
		        rate.sleep()

	def DETACH(self):
		#Mother and Daughter combined should stop. and Daughter should Takeoff. 
		pass 

	def REVERSELAP(self):
		#A copy of lap but with the setpoints list reversed.
		pass

	def PARENTLAND(self):	# Lands at current position.
		self.parent.gotopose(self.curr_x, self.curr_y, 0.1)
		print('MOTHERSHIP LANDED')

	def CHILDLAND(self):		#TODO : Change the Publisher
		self.child.gotopose(self.curr_x, self.curr_y, 0.1)
		print('DAUGHTERSHIP LANDED')

	def FINDER(self, x, y, z):
		# This State is when the daughter ship looks for the mast
		# TODO : Fix the Publisher
		self.child.gotopose(x + 10 + 5, y, z + 5)
		self.child.gotopose(x + 10, y, z)
		rate = rospy.Rate(20)
		while (self.isModuleDetected != True):
			self.child.circle(x, y, z)
			rate.sleep()
		self.APPROACH(self)
		#DONE

	def APPROACH(self):
		# This State is when the daughter ship approaches the module once it has detected it
		i = 0
		
		self.DANCE(self)
		# Write the Code for Detecting the Normal Vector from the mast and 
		# Approach the mast on that location.

	def DANCE(self):
		# Child Drone attempts to grab hold of the mast
		pass

	def REPLACE(self):
		# This state is when the module is actually replaced. 
		pass

	def GETOUT(self):
		self.child.gotopose(self.child.curr_x - 5, self.curr_y + 5, 10)
		#DONE

if __name__ == '__main__':
  rospy.init_node('autopilot_node', anonymous=True)
  autopilot = autopilot()
  #child = child()
  parent = parent()
  rospy.sleep(10)
  parent.setarmp(1)
  parent.offboard(0)
  parent.gotopose(0, 0, 5)

  # thread.start_new_thread( child.setarmc(1), 1 )
  # thread.start_new_thread( parent.setarmp(1), 1 )
  # stat_procs = []
  # p2 = Process(target=parent.setarmp(1))
  # p1 = Process(target=child.setarmc(1))

  # stat_procs.append(p1)
  # stat_procs.append(p2)

  # p1.start()
  # p2.start()

  # make_vectorized_agents([r1, r2]) 
  # p2.start()
  # p1.start()
  
  # p1.join()
  # p2.join()

  # p2 = Thread(target = parent.setarmp(1))
  # p1 = Thread(target = child.setarmc(1))
  # p1.start()
  # p2.start()

  