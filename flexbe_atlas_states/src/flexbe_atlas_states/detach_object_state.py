#!/usr/bin/env python

from flexbe_core import EventState, Logger

from geometry_msgs.msg import PoseStamped

from vigir_object_template_msgs.srv import *

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 04/21/2015

@author: Spyros Maniatopoulos
'''
class DetachObjectState(EventState):
	'''
	Requests that a template be detached from the robot.

	># template_id 		int 		ID of the template to detach from the robot.
	># templase_pose 	PoseStamped	The current pose of the template to detach.

	<= done 				Template has been detached.
	<= failed 				Failed to detach template.

	'''

	def __init__(self):
		'''Constructor'''
		super(DetachObjectState, self).__init__(outcomes = ['done', 'failed'],
												input_keys = ['template_id',
															  'template_pose'])

		self._service_topic = "/detach_object_template"
		# This service uses the same srv definition as the attach template one:
		self._srv = ProxyServiceCaller({self._service_topic: SetAttachedObjectTemplate})

		self._failed = False


	def execute(self, userdata):

		if self._failed:
			return 'failed'
		else:	
			return 'done'


	def on_enter(self, userdata):

		self._failed = False

		try:
			print 'Detaching template at:', userdata.template_pose
			request = SetAttachedObjectTemplateRequest(template_id = userdata.template_id,
													   pose = userdata.template_pose)
			self._srv.call(self._service_topic, request)
		except Exception as e:
			Logger.logwarn('Failed to send service request')
			rospy.logwarn(str(e))
			self.failed = True
