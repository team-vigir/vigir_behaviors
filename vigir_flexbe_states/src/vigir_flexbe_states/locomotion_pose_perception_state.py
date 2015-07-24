#!/usr/bin/env python

from flexbe_core import EventState, Logger

import actionlib
import rospy

from vigir_perception_msgs.msg import *


'''
Created on 10/22/2014

@author: Philipp Schillinger
'''

class LocomotionPosePerceptionState(EventState):
	'''
	Extracts a pose of interest to walk to from environment data.
	Needs to be tested and maybe updated.

	-- id_string    string          Identifier of the request. (?)

	#> target_pose  PoseStamped     Pose to plan to.

	<= detected                     Was able to detect the requested object and can provide a valid pose.
	<= failed                       Failed to detect an object of interest.

	'''


	def __init__(self, id_string = ""):
		'''
		Constructor
		'''
		super(LocomotionPosePerceptionState, self).__init__(outcomes=['detected', 'failed'],
													output_keys=['target_pose'])

		self._action_topic = "/worldmodel_main/get_locomotion_target_pose"

		self._client = actionlib.SimpleActionClient(self._action_topic, GetLocomotionTargetPoseAction)
		self._client.wait_for_server(rospy.Duration.from_sec(10))

		self._id_string = id_string

		self._failed = False


	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'

		if self._client.wait_for_result(rospy.Duration.from_sec(0.1)):
			result = self._client.get_result()
			target_pose = result.target_pose
			userdata.target_pose = target_pose
			if target_pose is None:
				return 'failed'
			else:
				return 'detected'


	def on_enter(self, userdata):
		action_goal = GetLocomotionTargetPoseGoal()
		action_goal.id_string = self._id_string

		try:
			self._client.send_goal(action_goal)
		except Exception as e:
			Logger.logwarn('Was unable to create pose perception request:\n%s' % str(e))
			self._failed = True







