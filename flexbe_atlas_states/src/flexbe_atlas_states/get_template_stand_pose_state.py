#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from vigir_object_template_msgs.srv import GetTemplateStateAndTypeInfo, GetTemplateStateAndTypeInfoRequest

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 02/24/2015

@author: Philipp Schillinger
'''
class GetTemplateStandPoseState(EventState):
	'''
	Requests the pose where to stand in front of a template from the template server.

	># template_id 	int 			ID of the template to get the stand pose for.
	># hand_side 	string 			Hand side for which the state will ask the template server for poses {left, right, both}
	># preference 	int 			Index of the preferred stand pose.

	#> grasp 		PoseStamped 	Pelvis pose of the robot required to perform grasping.

	<= done 						Pose data is available.
	<= failed 						Failed to get stand pose.
	<= not_available				The requested preference is not available (index too high).

	'''

	def __init__(self):
		'''Constructor'''
		super(GetTemplateStandPoseState, self).__init__(outcomes = ['done', 'failed', 'not_available'],
														input_keys = ['template_id', 'hand_side', 'preference'],
														output_keys = ['stand_pose'])

		self.hand_side = int()

		self._service_topic = "/template_info"
		self._srv = ProxyServiceCaller({self._service_topic: GetTemplateStateAndTypeInfo})

		self._srv_result = None
		self._hand_side = None

		self._failed = False
		self._done = False
		self._not_available = False


	def execute(self, userdata):

		if self._failed or self._srv_result is None:
			return 'failed'
		
		if self._done:
			return 'done'

		if self._not_available:
			return 'not_available'

		try:
			choices = self._srv_result.template_type_information.stand_poses
			Logger.loginfo("There are %d stand pose choices and want to select entry %d" % (len(choices), userdata.preference))
			if len(choices) <= userdata.preference:
				self._not_available = True
				return 'not_available'
			userdata.stand_pose = choices[userdata.preference].pose
		except Exception as e:
			Logger.logwarn('Failed to retrieve pose information from service result:\n%s' % str(e))
			self._failed = True
			return 'failed'
		
		self._done = True
		return 'done'


	def on_enter(self, userdata):
		
		self._failed = False
		self._done = False
		self._not_available = False

		if userdata.hand_side == 'left':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.LEFT_HAND
		elif userdata.hand_side == 'right':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.RIGHT_HAND
		elif userdata.hand_side == 'both':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.BOTH_HANDS
		else:
			Logger.logwarn('Unexpected value of hand side: %s Expected {left, right, both}' % str(userdata.hand_side))

		try:
			self._srv_result = self._srv.call(self._service_topic, GetTemplateStateAndTypeInfoRequest(userdata.template_id, self._hand_side))
		
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True
