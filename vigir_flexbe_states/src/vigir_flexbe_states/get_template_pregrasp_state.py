#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from vigir_object_template_msgs.srv import GetInstantiatedGraspInfo, GetInstantiatedGraspInfoRequest

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 04/02/2015

@author: Spyros Maniatopoulos
'''
class GetTemplatePregraspState(EventState):
	'''
	Requests pregrasp information for a template from the template server.

	># template_id 	int 			ID of the template to grasp.
	># hand_side 	string 			Hand side for which the state will ask the template server for pregrasp poses {left, right, both}
	># preference 	int 			Index of the preferred pregrasp.

	#> pre_grasp 	PoseStamped 	Target endeffector pose for pregrasp.

	<= done 						Pregrasp data is available.
	<= failed 						Failed to get pregrasp information.
	<= not_available				The requested preference is not available (index too high).

	'''

	def __init__(self):
		'''Constructor'''
		super(GetTemplatePregraspState, self).__init__(outcomes = ['done', 'failed', 'not_available'],
														input_keys = ['template_id', 'hand_side', 'preference'],
														output_keys = ['pre_grasp'])

		self._service_topic = "/instantiated_grasp_info"
		self._srv = ProxyServiceCaller({self._service_topic: GetInstantiatedGraspInfo})

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
			options = self._srv_result.pre_grasp_information.grasps
			if len(options) <= userdata.preference:
				Logger.logwarn('The option with index %d is not available. There are %d options total.' % (userdata.preference, len(options)))
				self._not_available = True
				return 'not_available'
			chosen_pregrasp = options[userdata.preference]
			userdata.pre_grasp = chosen_pregrasp.grasp_pose
			Logger.loginfo('Using grasp with ID: %s' % str(chosen_pregrasp.id))
		
		except Exception as e:
			Logger.logwarn('Failed to retrieve grasp information from service result:\n%s' % str(e))
			self._failed = True
			return 'failed'
		
		self._done = True
		return 'done'


	def on_enter(self, userdata):

		self._failed = False
		self._done = False
		self._not_available = False

		if userdata.hand_side == 'left':
			self._hand_side = GetInstantiatedGraspInfoRequest.LEFT_HAND
		elif userdata.hand_side == 'right':
			self._hand_side = GetInstantiatedGraspInfoRequest.RIGHT_HAND
		elif userdata.hand_side == 'both':
			self._hand_side = GetInstantiatedGraspInfoRequest.BOTH_HANDS
		else:
			Logger.logwarn('Unexpected value of hand side: %s Expected {left, right, both}' % str(userdata.hand_side))

		try:
			self._srv_result = self._srv.call(self._service_topic, GetInstantiatedGraspInfoRequest(int(userdata.template_id), self._hand_side))
		
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True