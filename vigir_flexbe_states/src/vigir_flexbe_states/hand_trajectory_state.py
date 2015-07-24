#!/usr/bin/env python

import os
import math

import rospy
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

'''
Created on 01/23/2015

@author: Spyros Maniatopoulos
'''
class HandTrajectoryState(EventState):
	'''
	Executes a given hand trajectory, i.e., a request to open or close the fingers.

	-- hand_type			string 			Type of hand (e.g. 'robotiq')

	># finger_trajectory 	JointTrajectory	A single joint trajectory for the hand joints.
	># hand_side			string 			Which hand side the trajectory refers to (e.g. 'left')

	<= done 								The trajectory was successfully executed.
	<= failed 								Failed to execute trajectory.

	'''

	def __init__(self, hand_type):
		'''Constructor'''
		super(HandTrajectoryState, self).__init__(outcomes=['done', 'failed'],
												  input_keys=['finger_trajectory', 'hand_side'])

		if not rospy.has_param("behavior/hand_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/hand_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/hand_controllers_name")

		if not rospy.has_param("behavior/hand_type_prefix"):
			Logger.logerr("Need to specify parameter behavior/hand_type_prefix at the parameter server")
			return

		hand_type_prefix = rospy.get_param("behavior/hand_type_prefix")

		# Determine which hand types and sides have been sourced
		self._hands_in_use = list()

		LEFT_HAND  = os.environ[hand_type_prefix + 'LEFT_HAND_TYPE']
		RIGHT_HAND = os.environ[hand_type_prefix + 'RIGHT_HAND_TYPE']

		if LEFT_HAND == 'l_' + hand_type:
			self._hands_in_use.append('left')
		if RIGHT_HAND == 'r_' + hand_type:
			self._hands_in_use.append('right')

		if len(self._hands_in_use) == 0:
			Logger.logerr('No %s hands seem to be in use:\nLEFT_HAND = %s\nRIGHT_HAND = %s' % (hand_type, LEFT_HAND, RIGHT_HAND))

		# Initialize the action clients corresponding to the hands in use
		self._client_topics = dict()
		self._active_topic = None
		self._client = ProxyActionClient()
		
		for hand_side in self._hands_in_use:
			action_topic = ("/%s/%s_hand_traj_controller/follow_joint_trajectory" % (controller_namespace, hand_side))
			self._client.setupClient(action_topic, FollowJointTrajectoryAction)
			self._client_topics[hand_side] = action_topic

		self._failed = False
	
	def execute(self, userdata):
		'''During execution of the state, keep checking for action servers response'''

		if self._failed:
			return 'failed'

		if self._client.has_result(self._active_topic):
			result = self._client.get_result(self._active_topic)
			# Logger.loginfo('Action server says: %s' % result.error_code)
			
			if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
				return 'done'
			else:
				Logger.logwarn('Hand trajectory request failed to execute: %s (%d)' % (result.error_string, result.error_code))
				
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, create and send the action goal message'''

		self._failed = False

		if userdata.hand_side not in self._hands_in_use:
			Logger.logerr('Hand side from userdata (%s) does not match hands in use: %s' % (userdata.hand_side, self._hands_in_use))
			self._failed = True
			return

		# Create goal message
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = userdata.finger_trajectory

		# Send goal to action server for execution
		try: 
			self._active_topic = self._client_topics[userdata.hand_side]
			self._client.send_goal(self._active_topic, goal)
		except Exception as e:
			Logger.logwarn('Failed to send follow (hand) joint trajectory action goal:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._active_topic):
			self._client.cancel(self._active_topic)
			Logger.loginfo("Cancelled active action goal.")
