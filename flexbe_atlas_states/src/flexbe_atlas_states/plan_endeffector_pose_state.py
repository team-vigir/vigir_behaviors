#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_atlas_states.proxy import ProxyMoveitClient

'''
Created on 24.11.2013

@author: Philipp Schillinger
'''

class PlanEndeffectorPoseState(EventState):
	'''
	Plans to reach the given pose with the specified endeffector.

	-- ignore_collisions 	boolean 			Should collisions be ignored? Only pass True if you are sure that it is safe.
	-- include_torso		boolean 			Should the torso be included in the trajectory or stay fixed?
	-- allowed_collisions	tuple[] 			List of 2-tuples (strings) for each pair of links which is allowed to collide.
												Set the second value to an empty string if collision to any other link is allowed.
	-- planner_id           string              Sets the ID of the planner to use (MoveIt planner id or "drake" - default = "RRTConnectkConfigDefault")


	># target_pose 			PoseStamped 		Target pose to reach.
	># hand 				string 				One of the class constants determining the hand to be moved.

	#> joint_trajectory 	JointTrajectory 	Trajectory of the end effector to perform the requested motion.

	<= planned 									Was able to generate a valid motion plan.
	<= failed 									Failed to create a motion plan at all.

	'''
	
	LEFT_HAND = 'left'
	RIGHT_HAND = 'right'
	

	def __init__(self, ignore_collisions = False, include_torso = False, allowed_collisions = [], planner_id = "RRTConnectkConfigDefault"):
		'''
		Constructor
		'''
		super(PlanEndeffectorPoseState, self).__init__(outcomes=['planned', 'failed'],
												   input_keys=['target_pose', 'hand'],
												   output_keys=['joint_trajectory'])
		
		self._client = ProxyMoveitClient()
		
		self._ignore_collisions = ignore_collisions
		self._include_torso = include_torso
		self._allowed_collisions = allowed_collisions
		self._failed = False
		self._planner_id = planner_id
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'

		if self._client.finished():
			userdata.joint_trajectory = self._client.get_plan()
			if self._client.success():
				return 'planned'
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
		
	
	def on_enter(self, userdata):
		if not hasattr(userdata, 'target_pose') or userdata.target_pose is None:
			self._failed = True
			Logger.logwarn('Userdata "target_pose" of state %s does not exist or is currently undefined!' % self.name)
			return
		if not hasattr(userdata, 'hand') or userdata.hand is None:
			self._failed = True
			Logger.logwarn('Userdata "hand" of state %s does not exist or is currently undefined!' % self.name)
			return
		self._failed = False

		planning_group_str = \
			('l' if userdata.hand == self.LEFT_HAND else 'r') + \
			'_arm' + \
			('_with_torso' if self._include_torso else '') + \
			'_group'

		# create the motion goal
		self._client.new_goal(planning_group_str)
		self._client.add_endeffector_pose(userdata.target_pose.pose, userdata.target_pose.header.frame_id)
		self._client.set_collision_avoidance(self._ignore_collisions)
		self._client.set_planner_id(self._planner_id)
		
		for link, target in self._allowed_collisions:
			self._client.set_allowed_collision(link, target)

		# for later use
		#for link, target in self._allowed_collisions:	
		
		Logger.loginfo('Sending planning request to reach point (%f, %f, %f) in frame %s...' % (userdata.target_pose.pose.position.x, userdata.target_pose.pose.position.y, userdata.target_pose.pose.position.z, userdata.target_pose.header.frame_id))
		
		try:
			self._client.start_planning()
		except Exception as e:
			Logger.logwarn('Could not request a plan!\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.finished():
			self._client.cancel()
			Logger.loginfo("Cancelled active action goal.")
		
		
	
	



