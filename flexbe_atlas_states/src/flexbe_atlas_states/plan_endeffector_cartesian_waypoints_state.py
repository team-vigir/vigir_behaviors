#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_atlas_states.proxy import ProxyMoveitClient

'''
Created on 30.11.2013

@author: Philipp Schillinger
'''

class PlanEndeffectorCartesianWaypointsState(EventState):
	'''
	Plans a cartesian endeffector trajectory passing all given waypoints.

	-- ignore_collisions 			boolean 			Should collisions be ignored? Only pass True if you are sure that it is safe.
	-- include_torso				boolean 			Should the torso be included in the trajectory or stay fixed?
	-- keep_endeffector_orientation	boolean 			Should the initial orientation of the endeffector be kept?
	-- allow_incomplete_plans		boolean 			Should incomplete plans be allowed?
	-- vel_scaling  				string 				Scales the velocity of the motion, lower values for slower trajectories.
	-- planner_id           string              Sets the ID of the planner to use (MoveIt planner id or "drake" - default = "RRTConnectkConfigDefault")

	># waypoints 					Pose[] 				Waypoints for the endeffector motion.
	># hand 						string 				One of the class constants determining the hand to be moved.
	># frame_id 					string 				Frame in which the waypoints are given.

	#> joint_trajectory 			JointTrajectory 	Trajectory of the endeffector to perform the requested motion.
	#> plan_fraction 				float 				Fraction of the requested motion which could be planned.
														A value below 1 indicates a partial motion.

	<= planned 											Was able to generate a valid motion plan.
														This plan is not necessarily complete.
	<= failed 											Failed to create a motion plan at all.
	
	'''

	LEFT_HAND = 'left'
	RIGHT_HAND = 'right'
	

	def __init__(self, ignore_collisions = False,
				 include_torso = False,
				 keep_endeffector_orientation = False,
				 allow_incomplete_plans = False,
				 vel_scaling = 0.1,
				 planner_id = "RRTConnectkConfigDefault"):
		'''Constructor'''
		
		super(PlanEndeffectorCartesianWaypointsState, self).__init__(outcomes=['planned', 'incomplete', 'failed'],
												   					 input_keys=['waypoints', 'hand', 'frame_id'],
												   					 output_keys=['joint_trajectory', 'plan_fraction'])
		
		self._client = ProxyMoveitClient()
		
		self._ignore_collisions = ignore_collisions
		self._include_torso = include_torso
		self._keep_endeffector_orientation = keep_endeffector_orientation
		self._allow_incomplete_plans = allow_incomplete_plans
		self._vel_scaling = vel_scaling
		self._planner_id = planner_id
		
		self._failed = False
		self._done = False
		self._incomplete = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''

		fail_threshold = 0.05
		success_threshold = 0.95

		if self._failed:
			return 'failed'
		if self._done:
			return 'planned'
		if self._incomplete:
			return 'incomplete'

		if self._client.finished():
			userdata.joint_trajectory = self._client.get_plan()
			
			if self._client.success():
				# Incomplete plans are considered a SUCCESS, if we allow them
				plan_fraction = self._client.get_plan_fraction()
				Logger.loginfo('Plan completion fraction = %.2f%%' % (plan_fraction*100))
				userdata.plan_fraction = plan_fraction

				if plan_fraction < fail_threshold:
					rospy.logwarn('Failure! The plan fraction was below %d%%' % (fail_threshold*100))
					self._failed = True
					return 'failed'
				elif plan_fraction > success_threshold:
					rospy.loginfo('Success! The plan fraction was above %d%%' % (success_threshold*100))
					self._done = True
					return 'planned'
				else:
					rospy.logwarn('Partial success.The plan fraction was between %d%% and %d%%' % (fail_threshold*100, success_threshold*100))
					self._incomplete = True
					return 'incomplete'
			
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
		
	
	def on_enter(self, userdata):
		'''...'''

		if not hasattr(userdata, 'waypoints') or userdata.waypoints is None \
		or not hasattr(userdata, 'hand') or userdata.hand is None \
		or not hasattr(userdata, 'frame_id') or userdata.frame_id is None:
			self._failed = True
			Logger.logwarn('Userdata key of state %s does not exist or is currently undefined!' % self.name)
			return
		
		self._failed = False
		self._done = False
		self._incomplete = False

		planning_group_str = \
			('l' if userdata.hand == self.LEFT_HAND else 'r') + \
			'_arm' + \
			('_with_torso' if self._include_torso else '') + \
			'_group'

		# create the motion goal
		self._client.new_goal(planning_group_str)
		self._client.set_collision_avoidance(self._ignore_collisions)
		self._client.set_cartesian_motion()
		self._client.set_keep_orientation(self._keep_endeffector_orientation)
		self._client.set_execute_incomplete_plans(self._allow_incomplete_plans)
		self._client.set_velocity_scaling(self._vel_scaling)
		self._client.set_planner_id(self._planner_id)

		for pose in userdata.waypoints:
			self._client.add_endeffector_pose(pose, userdata.frame_id)

		rospy.loginfo('Sending planning request to pass %d waypoints in frame %s...' % (len(userdata.waypoints), userdata.frame_id))
		
		try:
			self._client.start_planning()
		except Exception as e:
			Logger.logwarn('Could not request a plan!\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.finished():
			self._client.cancel()
			Logger.loginfo("Cancelled active action goal.")
		
		
	
	



