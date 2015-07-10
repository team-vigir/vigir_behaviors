#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_atlas_states.proxy import ProxyMoveitClient

'''
Created on 30.11.2013

@author: Philipp Schillinger
'''

class PlanEndeffectorCircularMotionState(EventState):
	'''
	Plans a circular motion of the endeffector.

	-- ignore_collisions 				boolean 			Should collisions be ignored? Only pass True if you are sure that it is safe.
	-- include_torso					boolean 			Should the torso be included in the trajectory or stay fixed?
	-- keep_endeffector_orientation		boolean 			Should the initial orientation of the endeffector be kept?
	-- planner_id           string              Sets the ID of the planner to use (MoveIt planner id or "drake" - default = "RRTConnectkConfigDefault")


	># hand 							string 				One of the class constants determining the hand to be moved.
	># rotation_axis 					PoseStamped			Pose in the center of the rotation, can be retrieved from a template pose.
	># rotation_angle 					float 				Angle of the rotation in radians.

	#> joint_trajectory 				JointTrajectory 	Trajectory of the endeffector to perform the requested motion.
	#> plan_fraction 					float 				Fraction of the requested motion which could be planned.
															A value below 1 indicates a partial motion.

	<= planned 												Was able to generate a valid motion plan.
															This plan is not necessarily complete.
	<= failed 												Failed to create a motion plan at all.
	
	'''
	
	LEFT_HAND = 'left'
	RIGHT_HAND = 'right'
	

	def __init__(self, ignore_collisions = False, include_torso = False, keep_endeffector_orientation = False, planner_id = "RRTConnectkConfigDefault"):
		'''
		Constructor
		'''
		super(PlanEndeffectorCircularMotionState, self).__init__(outcomes=['planned', 'failed'],
												   input_keys=['hand', 'rotation_axis', 'rotation_angle'],
												   output_keys=['joint_trajectory', 'plan_fraction'])
		
		self._client = ProxyMoveitClient()
		
		self._ignore_collisions = ignore_collisions
		self._include_torso = include_torso
		self._keep_endeffector_orientation = keep_endeffector_orientation
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
				userdata.plan_fraction = 1
				return 'planned'
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
		
	
	def on_enter(self, userdata):
		if not hasattr(userdata, 'rotation_axis') or userdata.rotation_axis is None \
		or not hasattr(userdata, 'hand') or userdata.hand is None \
		or not hasattr(userdata, 'rotation_angle') or userdata.rotation_angle is None:
			self._failed = True
			Logger.logwarn('Userdata key of state %s does not exist or is currently undefined!' % self.name)
			return
		
		center_pose = userdata.rotation_axis
		planning_group_str = \
			('l' if userdata.hand == self.LEFT_HAND else 'r') + \
			'_arm' + \
			('_with_torso' if self._include_torso else '') + \
			'_group'

		# create the motion goal
		self._client.new_goal(planning_group_str)
		self._client.set_collision_avoidance(self._ignore_collisions)
		self._client.set_circular_motion(userdata.rotation_angle)
		self._client.set_keep_orientation(self._keep_endeffector_orientation)
		self._client.add_endeffector_pose(userdata.rotation_axis.pose, userdata.rotation_axis.header.frame_id) # rotation center
		self._client.set_planner_id(self._planner_id)
		
		Logger.loginfo('Sending planning request for circular motion of %f rad...' % userdata.rotation_angle)
		
		try:
			self._client.start_planning()
		except Exception as e:
			Logger.logwarn('Could not request a plan!\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.finished():
			self._client.cancel()
			Logger.loginfo("Cancelled active action goal.")
		
		
	
	



