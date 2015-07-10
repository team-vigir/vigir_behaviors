#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from vigir_object_template_msgs.srv import GetTemplateStateAndTypeInfo, GetTemplateStateAndTypeInfoRequest

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 02/20/2015

@author: Philipp Schillinger
'''
class GetTemplateGraspState(EventState):
	'''
	Requests a grasp for a template from the template server.

	># template_id 	int 			ID of the template to grasp.
	># hand_side 	string 			Hand side for which the state will ask the template server for poses {left, right, both}
	># preference 	int 			Index of the preferred grasp.

	#> grasp 		PoseStamped 	Target endeffector pose for grasping.

	<= done 						Grasp data is available.
	<= failed 						Failed to get grasping information.
	<= not_available				The requested preference is not available (index too high).

	'''

	def __init__(self):
		'''Constructor'''
		super(GetTemplateGraspState, self).__init__(outcomes = ['done', 'failed', 'not_available'],
														input_keys = ['template_id', 'hand_side', 'preference'],
														output_keys = ['grasp'])

		self._service_topic = "/template_info"
		self._srv = ProxyServiceCaller({self._service_topic: GetTemplateStateAndTypeInfo})

		self._srv_result = None
		self._hand_side = None

		self._failed = False
		self._done = False


	def execute(self, userdata):

		if self._failed or self._srv_result is None:
			return 'failed'
		if self._done:
			return 'done'
		
		try:
			choices = self._srv_result.template_type_information.grasps
			if len(choices) <= userdata.preference:
				return 'not_available'
			chosen_grasp = choices[userdata.preference]
			userdata.grasp = chosen_grasp.grasp_pose
			Logger.loginfo('Using grasp with ID: %s' % str(chosen_grasp.id))
		
		except Exception as e:
			Logger.logwarn('Failed to retrieve grasp information from service result:\n%s' % str(e))
			self._failed = True
			return 'failed'
		
		self._done = True
		return 'done'


	def on_enter(self, userdata):

		self._failed = False
		self._done = False

		if userdata.hand_side == 'left':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.LEFT_HAND
		elif userdata.hand_side == 'right':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.RIGHT_HAND
		elif userdata.hand_side == 'both':
			self._hand_side = GetTemplateStateAndTypeInfoRequest.BOTH_HANDS
		else:
			Logger.logwarn('Unexpected value of hand side: %s Expected {left, right, both}' % str(userdata.hand_side))

		try:
			self._srv_result = self._srv.call(self._service_topic, GetTemplateStateAndTypeInfoRequest(int(userdata.template_id), self._hand_side))
		
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True