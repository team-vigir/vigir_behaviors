#!/usr/bin/env python

import rospy
import actionlib

from flexbe_core import EventState, Logger
from vigir_flexbe_states.proxy import ProxyMoveitClient

from moveit_msgs.msg import *
from vigir_planning_msgs.msg import *

"""
Created on 10/28/2014

@author: Philipp Schillinger
"""

class MoveitMoveGroupState(EventState):
	"""
	Uses moveit to plan to the given joint configuration and executes the resulting trajctory.

	-- planning_group 		string 		Name of the planning group.

	># target_joint_config	float[] 	Target joint values to plan and move to.

	<= reached 							Requested joint configuration has been reached successfully.
	<= failed 							Failed to reach requested joint configuration.

	"""
	
	def __init__(self, planning_group):
		"""Constructor"""
		super(MoveitMoveGroupState, self).__init__(outcomes=['reached', 'failed'],
												input_keys=['target_joint_config'])

		self._client = ProxyMoveitClient()

		self._failed = False
		self._planning_group = planning_group


	def execute(self, userdata):
		"""Execute this state"""
		if self._failed:
			return 'failed'

		if self._client.finished():
			if self._client.success():
				return 'reached'
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		self._failed = False

		# create the motion goal
		self._client.new_goal(self._planning_group)
		self._client.add_joint_values(userdata.target_joint_config)

		# execute the motion
		try: 
			Logger.loginfo("Moving %s to: %s" % (self._planning_group, ", ".join(map(str, userdata.target_joint_config))))
			self._client.start_execution()
		except Exception as e:
			Logger.logwarn('Was unable to execute move group request:\n%s' % str(e))
			self._failed = True
