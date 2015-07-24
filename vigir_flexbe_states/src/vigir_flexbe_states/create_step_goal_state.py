#!/usr/bin/env python

import actionlib
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from vigir_footstep_planning_msgs.msg import *
from std_msgs.msg import *

'''
Created on 05/05/2015

@author: Spyros Maniatopoulos
'''

class CreateStepGoalState(EventState):
	'''
	Creates a footstep goal from a desired pose.

	-- pose_is_pelvis 	boolean 		Set this to True if the pose is given
										as pelvis pose and not on the ground.

	># target_pose 		PoseStamped 	Pose to which the robot should walk.

	#> step_goal 		Feet 			Desired feet pose.

	<= done 							Successfully created a step goal.
	<= failed 							Failed to create a plan.

	'''

	def __init__(self, pose_is_pelvis = False):
		'''Constructor'''

		super(CreateStepGoalState, self).__init__(outcomes = ['done', 'failed'],
												  input_keys = ['target_pose'],
												  output_keys = ['step_goal'])

		self._action_topic = "/vigir/footstep_manager/generate_feet_pose"

		self._client = ProxyActionClient({self._action_topic: GenerateFeetPoseAction})

		self._pose_is_pelvis = pose_is_pelvis

		self._done   = False
		self._failed = False


	def execute(self, userdata):
		'''Execute this state'''
		
		if self._failed:
			return 'failed'
		if self._done:
			return 'done'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)

			if result.status.error == ErrorStatus.NO_ERROR:
				userdata.step_goal = result.feet
				self._done = True
				return 'done'
			else:
				Logger.logwarn('Step goal creation failed:\n%s' % result.status.error_msg)
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, request the feet pose.'''
		
		self._done   = False
		self._failed = False

		# Create request msg and action goal
		request = FeetPoseRequest()
		request.header = userdata.target_pose.header
		request.header.stamp = rospy.Time.now() # timestamp used to track goal
		request.pose   = userdata.target_pose.pose
		request.flags  = FeetPoseRequest.FLAG_PELVIS_FRAME if self._pose_is_pelvis else 0

		action_goal = GenerateFeetPoseGoal(request = request)

		# Send action goal
		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send step goal request')
			rospy.logwarn(str(e))
			self._failed = True


	def on_exit(self, userdata):
		'''Destructor'''
		
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")
