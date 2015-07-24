#!/usr/bin/env python

import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import *
from vigir_planning_msgs.msg import *

"""
Created on 02/27/2015

@author: Spyros Maniatopoulos
"""

class MoveItMoveGroupPlanState(EventState):
	"""
	Employs MoveIt! Move Group to get a plan to a goal configuration, but does not execute it.
	
	-- vel_scaling 	float 				Percentage of maximum velocity at which the trajectory will be executed (0, 1.0]

	># desired_goal JointTrajectory		A joint configuration in the form of a joint trajectory with only one point.

	#> plan_to_goal JointTrajectory 	A joint trajectory to the desired_goal.

	<= done 							A plan to the desired configuration was successfully calculated.
	<= failed 							Failed to calculate a plan to the desired configuration.

	"""
	
	def __init__(self, vel_scaling=0.1):
		"""Constructor"""
		super(MoveItMoveGroupPlanState, self).__init__(outcomes=['done', 'failed'],
													   input_keys=['desired_goal'],
													   output_keys=['plan_to_goal'])

		self._action_topic = "/vigir_move_group"

		self._client = ProxyActionClient({self._action_topic: MoveAction})

		self._vel_scaling = vel_scaling

		self._failed = False
		self._done = False

	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''
		
		if self._failed:
			return 'failed'
		if self._done:
			return 'done'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.error_code.val is MoveItErrorCodes.SUCCESS:
				userdata.plan_to_goal = result.planned_trajectory
				Logger.loginfo('Received a %s motion plan. Planning time = %s' % (str(type(result.planned_trajectory)), str(result.planning_time)))
				self._done = True
				return 'done'
			else:
				Logger.logwarn('Motion plan request failed with error code: %d' % result.error_code.val)
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		"""Upon entering the state, get the first trajectory point and request moveit motion plan to get there."""

		self._done = False
		self._failed = False
		
		if userdata.desired_goal is None:
			self._done = True
			userdata.plan_to_goal = None
			return

		# Create the motion goal (one for both arms)
		move_group_goal = MoveGoal()
		move_group_goal.request = MotionPlanRequest()
		
		# This state will only request a plan, not execute it
		move_group_goal.planning_options = PlanningOptions()
		move_group_goal.planning_options.plan_only = True

		# Figure out which planning group is needed
		# see planning_groups.srdf for available groups
		# Logger.loginfo('MoveItMoveGroupPlanState: %d Key(s) in userdata.desired_goal: "%s"' % (len(userdata.desired_goal), str(userdata.desired_goal.keys())))
		if len(userdata.desired_goal) is 1:
			if 'left_arm' in userdata.desired_goal.keys():
				self._planning_group = "l_arm_group"
			elif 'right_arm' in userdata.desired_goal.keys():
				self._planning_group = "r_arm_group"
			elif 'left_leg' in userdata.desired_goal.keys():
				self._planning_group = "l_leg_group"
			elif 'right_leg' in userdata.desired_goal.keys():
				self._planning_group = "r_leg_group"
			elif 'torso' in userdata.desired_goal.keys():
				self._planning_group = "torso_group"
			else:
				Logger.logwarn('MoveItMoveGroupPlanState: Unexpected key (1) in desired_goal: "%s". Either "left_arm/leg" or "right_arm/leg" was expected.' % str(userdata.desired_goal.keys()))
				self._failed = True
		elif len(userdata.desired_goal) is 2:
			if 'left_arm' in userdata.desired_goal.keys() and 'right_arm' in userdata.desired_goal.keys():
				self._planning_group = "both_arms_group"
			elif 'left_leg' in userdata.desired_goal.keys() and 'right_leg' in userdata.desired_goal.keys():
				self._planning_group = "both_legs_group"
			else:
				Logger.logwarn('MoveItMoveGroupPlanState: Unexpected keys (2): Either both legs or both arms expected. Got "%s"' % str(userdata.desired_goal.keys()))
				self._failed = True
		elif len(userdata.desired_goal) is 3:
			if 'left_arm' in userdata.desired_goal.keys() and 'right_arm' in userdata.desired_goal.keys() and 'torso' in userdata.desired_goal.keys():
				self._planning_group = "both_arms_with_torso_group"
			else:
				Logger.logwarn('MoveItMoveGroupPlanState: Unexpected keys (3): Either both arms and torso. Got "%s"' % str(userdata.desired_goal.keys()))
				self._failed = True
		elif len(userdata.desired_goal) is 4:
			if 'left_arm' in userdata.desired_goal.keys() and 'right_arm' in userdata.desired_goal.keys() and 'left_leg' in userdata.desired_goal.keys() and 'right_leg' in userdata.desired_goal.keys():
				self._planning_group = "all_appendages_group"
			else:
				Logger.logwarn('MoveItMoveGroupPlanState: Unexpected keys (4): Expected both arms and legs. Got "%s"' % str(userdata.desired_goal.keys()))
				self._failed = True
		elif len(userdata.desired_goal) is 6:
			if 'left_arm' in userdata.desired_goal.keys() and 'right_arm' in userdata.desired_goal.keys() and 'left_leg' in userdata.desired_goal.keys() and 'right_leg' in userdata.desired_goal.keys() and 'torso' in userdata.desired_goal.keys() and 'neck' in userdata.desired_goal.keys():
				self._planning_group = "whole_body_group"
			else:
				Logger.logwarn('MoveItMoveGroupPlanState: Unexpected keys (6): Expected both arms, legs, neck, torso. Got "%s"' % str(userdata.desired_goal.keys()))
				self._failed = True
		else:
			Logger.logwarn('MoveItMoveGroupPlanState: Unexpected number (%d) of desired goal chain keys: "%s"' % (len(userdata.desired_goal), str(userdata.desired_goal.keys())))
			self._failed = True
		move_group_goal.request.group_name = self._planning_group
		move_group_goal.request.max_velocity_scaling_factor = self._vel_scaling

		# Get starting point and write to motion plan request
		move_group_goal.request.goal_constraints = [Constraints()]
		move_group_goal.request.goal_constraints[0].joint_constraints = []

		for chain, traj in userdata.desired_goal.iteritems():

			joint_names = traj.joint_names

			starting_point = traj.points[0].positions # Get first/starting trajectory point

			if len(joint_names) != len(starting_point):
				Logger.logwarn("MoveItMoveGroupPlanState: Number of joint names (%d) does not match number of joint values (%d)" % (len(joint_names), len(starting_point)))

			for i in range(min(len(joint_names), len(starting_point))):
				move_group_goal.request.goal_constraints[0].joint_constraints.append(JointConstraint(joint_name = joint_names[i], 
																									 position = starting_point[i])
				)

		# Send the motion plan request to the server
		try: 
			Logger.loginfo("MoveItMoveGroupPlanState: Motion plan for %s has been requested." % self._planning_group)
			self._client.send_goal(self._action_topic, move_group_goal)
		except Exception as e:
			Logger.logwarn('MoveItMoveGroupPlanState: Was unable to request a motion plan for %s because:\n%s' % (self._planning_group, str(e)))
			self._failed = True
