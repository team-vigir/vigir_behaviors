#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from vigir_object_template_msgs.srv import *

'''
Created on 05/15/2015

@author: Spyros Maniatopoulos
'''
class GetTemplateUsabilityState(EventState):
	'''
	Requests a grasp for a template from the template server.

	-- usability_name	string		Name of the template usability (e.g. 'tip')
	-- usability_id 	int 		ID of the template usability.
									Set to 100 if requesting by name instead.

	># template_id 		int 		ID of the template of interest

	#> usability_pose 	PoseStamped Pose of the template's usability in the 
									reference frame of the wrist (r/l_hand)

	<= done 						The usability has been retrieved.
	<= failed 						Failed to get the template's usability.

	'''

	def __init__(self, usability_name = '', usability_id = 100):
		'''Constructor'''
		super(GetTemplateUsabilityState, self).__init__(outcomes = ['done', 'failed'],
														input_keys = ['template_id'],
														output_keys = ['usability_pose'])

		# Set up service for attaching object template
		self._service_topic = "/usability_pose"
		self._srv = ProxyServiceCaller({
						self._service_topic: GetUsabilityInWristFrame})

		self._usability_id = usability_id
		self._usability_name = usability_name

		self._failed = False


	def execute(self, userdata):
		'''Move along people, nothing to see here!'''

		if self._failed:
			return 'failed'
		else:
			return 'done'


	def on_enter(self, userdata):
		'''Send the usability request and write response to userdata.'''

		self._failed = False

		if self._usability_id != 100 and self._usability_name != '':
			Logger.logwarn('Not clear if you are requesting usability by ID (%d) + \
							or name (%s)' % (self._usability_id, self._usability_name))

		try:
			request = GetUsabilityInWristFrameRequest(
							template_id  = userdata.template_id,
							usability_id = self._usability_id,
							usability_name = self._usability_name)
			
			self._srv_result = self._srv.call(self._service_topic, request)
			userdata.usability_pose = self._srv_result.wrist_usability
			print request, self._srv_result.wrist_usability # debug
		
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True
			return
