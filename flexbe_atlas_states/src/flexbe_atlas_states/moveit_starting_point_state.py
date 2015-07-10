#!/usr/bin/env python

import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import *
from vigir_planning_msgs.msg import *

"""
Created on 02/18/2015

@author: Spyros Maniatopoulos
"""

class MoveitStartingPointState(EventState):
	"""
	Uses moveit to plan and move to the first point of a given arm trajectory.

	-- vel_scaling 		float 		Scales the velocity of the planned trajectory,
									lower values for slower trajectories.

	># trajectories		dict 		arm side (str) : joint values (float[])

	"""
	
	def __init__(self, vel_scaling = 0.1):
		"""Constructor"""
		super(MoveitStartingPointState, self).__init__(outcomes=['reached', 'failed'],
													   input_keys=['trajectories'])

		self._action_topic = "/vigir_move_group"

		self._client = ProxyActionClient({self._action_topic: MoveAction})

		self._vel_scaling = vel_scaling

		self._failed = False


	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''
		
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.error_code.val is MoveItErrorCodes.SUCCESS:
				return 'reached'
			else:
				Logger.logwarn('Motion plan request failed with the following error code: %d' % result.error_code.val)
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		"""Upon entering the state, get the first trajectory point and request moveit motion plan to get there."""

		self._failed = False

		# Create the motion goal (one for both arms)
		move_group_goal = MoveGoal()
		move_group_goal.request = MotionPlanRequest()

		# Figure out which planning group is needed
		if len(userdata.trajectories) is 2:
			self._planning_group = "both_arms_group"
		else:
			chain = userdata.trajectories.keys()[0]
			if chain is 'left_arm':
				self._planning_group = "l_arm_group"
			elif chain is 'right_arm':
				self._planning_group = "r_arm_group"
			if chain is 'left_leg':
				self._planning_group = "l_leg_group"
			elif chain is 'right_leg':
				self._planning_group = "r_leg_group"
			else:
				Logger.logwarn('Unexpected key: %s Either "left" or "right" was expected.' % arm)
		
		move_group_goal.request.group_name = self._planning_group
		move_group_goal.request.max_velocity_scaling_factor = self._vel_scaling

		# Get first/starting trajectory point
		move_group_goal.request.goal_constraints = [Constraints()]
		move_group_goal.request.goal_constraints[0].joint_constraints = []

		for arm, traj in userdata.trajectories.iteritems():

			joint_names = traj.joint_names

			starting_point = traj.points[0].positions

			if len(joint_names) != len(starting_point):
				Logger.logwarn("Number of joint names (%d) does not match number of joint values (%d)" % (len(joint_names), len(starting_point)))

			for i in range(min(len(joint_names), len(starting_point))):
				move_group_goal.request.goal_constraints[0].joint_constraints.append(JointConstraint(joint_name = joint_names[i], 
																									 position = starting_point[i])
				)

		# Send the motion plan request to the server
		try: 
			Logger.loginfo("Moving %s to starting point." % self._planning_group)
			self._client.send_goal(self._action_topic, move_group_goal)
		except Exception as e:
			Logger.logwarn('Was unable to execute %s motion request:\n%s' % (self._planning_group, str(e)))
			self._failed = True
