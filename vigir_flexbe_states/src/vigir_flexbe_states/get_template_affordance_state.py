#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from vigir_object_template_msgs.srv import GetTemplateStateAndTypeInfo, GetTemplateStateAndTypeInfoRequest

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 04/06/2015

@author: Philipp Schillinger
'''
class GetTemplateAffordanceState(EventState):
	'''
	Requests affordance information for a template from the template server.

	-- identifier 	string 		Name to identify the requested affordance.

	># template_id 	int 		ID of the template to manipulate.
	># hand_side 	string 		Hand side for which the state will ask the template server for affordances {left, right, both}

	#> affordance 	Affordance 	A message as defined in vigir_object_template_msgs containing all required information.

	<= done 					Affordance data is available.
	<= failed 					Failed to get affordance information.
	<= not_available			The requested preference is not available (index too high).

	'''

	def __init__(self, identifier):
		'''Constructor'''
		super(GetTemplateAffordanceState, self).__init__(outcomes = ['done', 'failed', 'not_available'],
														input_keys = ['template_id', 'hand_side'],
														output_keys = ['affordance'])

		self._service_topic = "/template_info"
		self._srv = ProxyServiceCaller({self._service_topic: GetTemplateStateAndTypeInfo})

		self._srv_result = None
		self._identifier = identifier
		self._hand_side = None

		self._failed = False
		self._done = False


	def execute(self, userdata):

		if self._failed or self._srv_result is None:
			return 'failed'
		if self._done:
			return 'done'
		
		try:
			choices = self._srv_result.template_type_information.affordances
			if not self._identifier in [a.name for a in choices]:
				return 'not_available'
			chosen_affordance = next(a for a in choices if a.name == self._identifier)
			userdata.affordance = chosen_affordance
			Logger.loginfo('Using grasp with ID: %s' % str(chosen_affordance.id))
		
		except Exception as e:
			Logger.logwarn('Failed to retrieve affordance information from service result:\n%s' % str(e))
			self._failed = True
			return 'failed'
		
		self._done = True
		return 'done'


	def on_enter(self, userdata):

		self._failed = False
		self._done = False
		self._srv_result = None

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