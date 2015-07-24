#!/usr/bin/env python

import os
import math
import rosbag
import actionlib
import rospy

from trajectory_msgs.msg import *

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
# from flexbe_behaviors.atlas_definitions import AtlasDefinitions

'''
Created on 04/26/2015

@author: Moritz Schappler
'''

class GenerateTrajectoryFromTxtfileState(EventState):
	'''
	Implements a state that loads (arm) trajectories stored in a text file.
	Copied from LoadTrajectoryFromBagfileState

	-- chains				list()		dict with file paths as values and chain names as keys
	-- transitiontime		int			Time between two positions
	-- settlingtime			int			Time after reaching a position
	
	># txtfilepaths			dict()		list with strings of joint chains like "left_arm", ...
	
	#> trajectories JointTrajectory{}	A dictionary where the keys are ['left_arm', 'right_arm'] and each has a trajectory as the value.

	<= done 							Trajectory has been successfully loaded from the bagfile.
	<= failed 							Failed to load trajectory.

	'''
	
	def __init__(self, chains, transitiontime, settlingtime):
		'''Constructor'''
		super(GenerateTrajectoryFromTxtfileState, self).__init__(outcomes = ['done', 'failed'], 
															input_keys = ['txtfilepaths'], 
															output_keys = ['trajectories'])

		self._trajectories = dict() # Store trajectories here until writing to userdata
		
		self._chains = chains
		self._transitiontime = transitiontime
		self._settlingtime = settlingtime

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
			#Logger.logwarn('Looks like the text files might have been empty!?')
			self._failed = True
			return 'failed'
	
	def on_enter(self, userdata):
		try:
			'''Upon entering the state, load trajectories from txtfile and write goal message.'''
	
			self._done = False
			self._failed = False
			self.txtfilepaths = userdata.txtfilepaths
	
			# Definitions
			l_arm_range = range(16,23);
			r_arm_range = range(23,30);
			atlasJointNames = [
			    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
			    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
			    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
			    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2',
			    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']
			
			# Set Trajectory Message Header
			# has to be zero, so that appended trajectories have the same starting time
			# has to be the same as below to support rosbag play
			#try:
			for chain in self._chains:
				t = 0.0 # time after start
				# Initialize Trajectory Message
				jt = JointTrajectory()
				jt.header.stamp = rospy.rostime.Time.from_sec(0.1)
				if chain == 'left_arm':
					joint_range = l_arm_range
				elif chain == 'right_arm':
					joint_range = r_arm_range
				else:
					Logger.logwarn('CalculateForceTorqueCalibration: Undefined chain %s', chain)
				txtfile = os.path.expanduser(self.txtfilepaths[chain])
				# Add Joint Names depending on chain
				for i in joint_range:
					jt.joint_names.append(atlasJointNames[i])
				line_number = 0
				with open(txtfile) as f:
					#Logger.loginfo('Accessing txtfile: %s' % txtfile)
					# Loop over lines and extract variables of interest
					for line in f:
						line_number = line_number + 1
						# get line of text file
						line = line.strip()
						columns = line.split()
						# Check number of columns
						if len(columns) != len(joint_range):
							Logger.logwarn('Input %s contains %d columns in line %d. Expected %d.' % (txtfile, len(columns), line_number, len(joint_range)) ) 
							self._failed = True
							return
						
						# assemble Joint Trajectory message
						for t_add in [self._transitiontime, self._settlingtime]:
							p = JointTrajectoryPoint()
							# Set Time
							t = float(t) + float(t_add)
							p.time_from_start = rospy.rostime.Time.from_sec(t)
	
							# append joint positions
							I=0 # Column-Index of text-File
							for j in range(len(joint_range)):
								p.positions.append(float(columns[I]))
								p.velocities.append(0.0)
								p.accelerations.append(0.0)
								p.effort.append(0.0)
						  		I=I+1
							# append point to trajectory
							jt.points.append(p)  
						
					# end trajectory points assembly
					self._trajectories[chain] = jt
				if line_number == 0:
					Logger.logwarn('Loaded only %d lines from %s. Something went wrong.' % (line_number, txtfile) ) 
					self._failed = True
					return
			userdata.trajectories = self._trajectories

		except Exception as e:
			Logger.logwarn('Could not load trajectory from text file because:\n %s' % str(e))
			self._failed = True