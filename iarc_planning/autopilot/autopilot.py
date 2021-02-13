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
from std_msgs.msg import Int32
import pymap3d as pm



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
		self.LAP()

	def LAP(self):
		lat0 = 47.3977418
		lon0 = 8.5455940
		h0 = 0
		# wplist = [[47.39765761,8.54567804,10],[47.39765051,8.5457109,10],[47.3976517,8.5490403,10],[47.397658728185355,8.549073930306008,10],[47.39767601073463,8.549099230295639,10],[47.39769957818326,8.54910992151855,10],[47.39771976,8.5490977,10],[47.39773617,8.54907284,10],[47.39774184,8.54903777,10],[47.39774191,8.54570904,10],[47.39773547,8.54567526,10],[47.39771924,8.54565179,10],[47.39769602,8.54564273,10],[47.39767368766275,8.545654009347942,10]]#*8
		wplist = [[47.397660849276726, 8.545655730049234, 10.0],[47.397660796492254, 8.549073402298644, 10.0],[47.39773275267587, 8.549073407035635, 10.0],[47.39773280546047, 8.545655730133277, 10.0]]
		for wp in wplist:
			x, y, z = pm.geodetic2enu(wp[0],wp[1],wp[2], lat0, lon0, h0)
			print(x, y, z)
			dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
			rate = rospy.Rate(20)
			while(dist>0.5):
				print(dist)
				self.parent.og_gotopose(x,y,z,10)
				dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
				rate.sleep()
		self.parent.check = 1
		self.DETACH()

	def DETACH(self):
		#Mother and Daughter combined should stop. and Daughter should Takeoff. 
		self.parent.gotopose(-6, 0, 10)
		time.sleep(20)
		self.REVERSELAP()
		# self.parent.setmode('POSCTL')

	def REVERSELAP(self):
		lat0 = 47.3977418
		lon0 = 8.5455940
		h0 = 0
		# wplist = [[47.39765761,8.54567804,10],[47.39765051,8.5457109,10],[47.3976517,8.5490403,10],[47.397658728185355,8.549073930306008,10],[47.39767601073463,8.549099230295639,10],[47.39769957818326,8.54910992151855,10],[47.39771976,8.5490977,10],[47.39773617,8.54907284,10],[47.39774184,8.54903777,10],[47.39774191,8.54570904,10],[47.39773547,8.54567526,10],[47.39771924,8.54565179,10],[47.39769602,8.54564273,10],[47.39767368766275,8.545654009347942,10]]#*8
		wplist = [[47.397660849276726, 8.545655730049234, 10.000008056735064],[47.397660796492254, 8.549073402298644, 10.005404873060154],[47.39773275267587, 8.549073407035635, 10.005398593771647],[47.39773280546047, 8.545655730133277, 10.00000177785658]]
		for wp in reversed(wplist):
			x, y, z = pm.geodetic2enu(wp[0],wp[1],wp[2], lat0, lon0, h0)
			print(x, y, z)
			dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
			rate = rospy.Rate(20)
			while(dist>0.5):
				print(dist)
				self.parent.og_gotopose(x,y,z,10)
				dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
				rate.sleep()
		self.parent.check = 1
		self.PARENTLAND()

	def PARENTLAND(self):	# Lands at current position.
		self.parent.gotopose(0, 0, 0.8)
		self.parent.setmode('AUTO.LAND')
		print('MOTHERSHIP LANDED')

	def CHILDLAND(self):		#TODO : Change the Publisher
		self.child.gotopose(self.curr_x, self.curr_y, 0.1)
		print('DAUGHTERSHIP LANDED')

	def FINDER(self, x, y, z):
		# This State is when the daughter ship looks for the mast
		# TODO : Fix the Publisher
		self.child.gotopose( -10, -8, 15)
		self.child.gotopose( -10, -3, 5)
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
  child = child()
  parent = parent()
  rospy.sleep(5)
  autopilot.TAKEOFF(10)

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

  