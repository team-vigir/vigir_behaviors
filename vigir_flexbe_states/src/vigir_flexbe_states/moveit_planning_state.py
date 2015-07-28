#!/usr/bin/env python

import rospy
import actionlib

from flexbe_core import EventState, Logger
from vigir_flexbe_states.proxy import ProxyMoveitClient

"""
Created on 04/08/2014

@author: Philipp Schillinger
"""

class MoveitPlanningState(EventState):
	"""
	Uses moveit to plan to the given joint configuration.

	-- planning_group 		string 				Name of the planning group.
	-- vel_scaling  		string 				Scales the velocity of the planned trajectory, lower values for slower trajectories.

	># joint_values			float[] 			Target joint values to plan to.

	#> joint_trajectory		JointTrajectory 	Planned joint trajectory to reach the goal.

	<= planned 									Requested joint trajectory has been plannd.
	<= failed 									Failed to plan to the requested joint configuration.

	"""
	
	def __init__(self, planning_group, vel_scaling=0.1):
		"""Constructor"""
		super(MoveitPlanningState, self).__init__(outcomes=['planned', 'failed'],
												input_keys=['joint_values'],
												output_keys=['joint_trajectory'])

		self._client = ProxyMoveitClient()

		self._planning_group = planning_group
		self._vel_scaling = vel_scaling
		self._failed = False


	def execute(self, userdata):
		"""Execute this state"""
		if self._failed:
			return 'failed'

		if self._client.finished():
			if self._client.success():
				userdata.joint_trajectory = self._client.get_plan()
				return 'planned'
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		self._failed = False

		# create the motion goal
		self._client.new_goal(self._planning_group)
		self._client.add_joint_values(userdata.joint_values)
		self._client.set_velocity_scaling(self._vel_scaling)

		# execute the motion
		try: 
			self._client.start_planning()
		except Exception as e:
			Logger.logwarn('Was unable to execute move group request:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.finished():
			self._client.cancel()
			Logger.loginfo("Cancelled active action goal.")
