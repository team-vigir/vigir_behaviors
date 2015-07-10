#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from vigir_object_template_msgs.srv import GetTemplateStateAndTypeInfo, GetTemplateStateAndTypeInfoRequest

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 02/20/2015

@author: Philipp Schillinger
'''
class GetTemplatePoseState(EventState):
	'''
	Requests the pose of a template from the template server.

	># template_id 	int 			ID of the template to get the pose for.

	#> grasp 		PoseStamped		The current pose of the template.

	<= done 						Pose data is available.
	<= failed 						Failed to get pose information.
	'''

	def __init__(self):
		'''Constructor'''
		super(GetTemplatePoseState, self).__init__(outcomes = ['done', 'failed'],
														input_keys = ['template_id'],
														output_keys = ['template_pose'])

		self._service_topic = "/template_info"
		self._srv = ProxyServiceCaller({self._service_topic: GetTemplateStateAndTypeInfo})

		self._srv_result = None

		self._failed = False


	def execute(self, userdata):

		if self._failed or self._srv_result is None:
			return 'failed'
		
		try:
			userdata.template_pose = self._srv_result.template_state_information.pose
		except Exception as e:
			Logger.logwarn('Failed to retrieve pose information from service result:\n%s' % str(e))
			return 'failed'
		return 'done'


	def on_enter(self, userdata):

		try:
			self._srv_result = self._srv.call(self._service_topic, GetTemplateStateAndTypeInfoRequest(userdata.template_id, 0)) # We don't care about the hand side
		
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True
