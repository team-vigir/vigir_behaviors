#!/usr/bin/env python

import os
import math
import rosbag
import actionlib
import rospy

from trajectory_msgs.msg import *

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

'''
Created on 02/18/2015

@author: Spyros Maniatopoulos
'''

class LoadTrajectoryFromBagfileState(EventState):
	'''
	Implements a state that loads (whole body) trajectories stored in a bagfile.

	># bagfile_name	string				Trajectory to be executed, given as list of time steps where each step contains a list of target joint values.
	
	#> trajectories JointTrajectory{}	A dictionary where the keys are ['left_arm', 'right_arm', 'left_leg', 'right_leg', 'torso'] and each has a trajectory as the value.

	<= done 							Trajectory has been successfully loaded from the bagfile.
	<= failed 							Failed to load trajectory.

	'''
	
	def __init__(self):
		'''Constructor'''
		super(LoadTrajectoryFromBagfileState, self).__init__(outcomes = ['done', 'failed'], 
															input_keys = ['bagfile_name'], 
															output_keys = ['trajectories'])

		self._trajectories = dict() # Store trajectories here until writing to userdata

		self._failed = False
		self._done = False

	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''
		
		if self._failed:
			return 'failed'
		if self._done:
			return 'done'
		
		if len(self._trajectories) != 0:
			self._done = True
			return 'done'
		else:
			Logger.logwarn('Looks like the bagfile might have been empty!?')
			self._failed = True
			return 'failed'
	
	def on_enter(self, userdata):
		'''Upon entering the state, load trajectories from bagfile and write goal message.'''

		self._done = False
		self._failed = False

		# Preliminaries
		bagfile = os.path.expanduser(userdata.bagfile_name)
		
		# Open bagfile, determine controller topic, read trajectories, write them to userdata
		try:

			with rosbag.Bag(bagfile) as bag:

				Logger.loginfo('Accessing bagfile: %s' % bagfile)

				# Get trajectory message(s) from bagfile
				for topic, msg, t in bag.read_messages():

					# The bagfile contains JointTrajectory messages
					trajectory = msg
					
					# Figure out which arm this trajectory is for
					if 'left_arm' in topic:
						chain = 'left_arm'
					elif 'right_arm' in topic:
						chain = 'right_arm'
					elif 'left_leg' in topic:
						chain = 'left_leg'
					elif 'right_leg' in topic:
						chain = 'right_leg'
					elif 'torso' in topic:
						chain = 'torso'
					else:
						Logger.logwarn('This is not a valid trajectory!? topic=%s' % str(topic))
						chain = ''

					# Write trajectory in a dictionary.
					if chain:
						self._trajectories[chain] = trajectory
						Logger.loginfo('Trajectoriy for chain %s added' % str(chain))

				userdata.trajectories = self._trajectories

		except Exception as e:
			Logger.logwarn('Could not load trajectory from bagfile because:\n %s' % str(e))
			self._failed = True
