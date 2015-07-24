#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import LaserScan

'''
Created on 03/04/2015

@author: Philipp Schillinger
'''
class GetLaserscanState(EventState):
	'''
	Grabs the most recent laserscan data.

	#> laserscan 	LaserScan 	The current laser scan.

	<= done 					Scanning data is available.

	'''

	def __init__(self):
		'''Constructor'''
		super(GetLaserscanState, self).__init__(outcomes = ['done'],
													output_keys = ['laserscan'])

		self._scan_topic = "/multisense/lidar_scan"
		self._sub = ProxySubscriberCached({self._scan_topic: LaserScan})


	def execute(self, userdata):

		if self._sub.has_msg(self._scan_topic):
			userdata.laserscan = self._sub.get_last_msg(self._scan_topic)
			return 'done'


	def on_enter(self, userdata):
		pass