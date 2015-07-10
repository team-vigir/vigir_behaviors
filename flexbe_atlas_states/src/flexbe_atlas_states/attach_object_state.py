#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from vigir_object_template_msgs.srv import *

'''
Created on 02/20/2015

@author: Philipp Schillinger
'''
class AttachObjectState(EventState):
	'''
	Requests a grasp for a template from the template server.

	># template_id 		int 		ID of the template to attach.
	># hand_side 		string 		Hand side to which the template
									should be attached {left, right}

	#> template_pose 	PoseStamped Pose of the attached template in the 
									reference frame of the wrist (r/l_hand)

	<= done 						Template has been attached.
	<= failed 						Failed to attach template.

	'''

	def __init__(self):
		'''Constructor'''
		super(AttachObjectState, self).__init__(outcomes = ['done', 'failed'],
												input_keys = ['template_id', 'hand_side'],
												output_keys = ['template_pose'])

		# Set up service for attaching object template
		self._service_topic = "/stitch_object_template"
		self._srv = ProxyServiceCaller({
						self._service_topic: SetAttachedObjectTemplate})

		# Set up subscribers for listening to the latest wrist poses
		self._wrist_pose_topics = dict()
		self._wrist_pose_topics['left']  = '/flor/l_arm_current_pose'
		self._wrist_pose_topics['right'] = '/flor/r_arm_current_pose'
		
		self._sub = ProxySubscriberCached({
						self._wrist_pose_topics['left']:  PoseStamped,
						self._wrist_pose_topics['right']: PoseStamped})
		
		self._sub.make_persistant(self._wrist_pose_topics['left'])
		self._sub.make_persistant(self._wrist_pose_topics['right'])

		self._failed = False


	def execute(self, userdata):

		if self._failed:
			return 'failed'
			
		return 'done'


	def on_enter(self, userdata):

		self._failed = False

		# The frame ID to which the object template will be attached to
		if userdata.hand_side == 'left':
			frame_id = "l_hand"
		elif userdata.hand_side == 'right':
			frame_id = "r_hand"
		else:
			Logger.logwarn('Unexpected value of hand side: %s Expected {left, right}' % str(userdata.hand_side))
			self._failed = True
			return

		# Get the current wrist pose, which is required by the stitching service
		try:
			wrist_pose_topic   = self._wrist_pose_topics[userdata.hand_side]
			current_wrist_pose = self._sub.get_last_msg(wrist_pose_topic)
			# Set the PoseStamped message's frame ID to the one
			# that the object template should be attached to:
			current_wrist_pose.header.frame_id = frame_id
		except Exception as e:
			Logger.logwarn('Failed to get the latest hand pose:\n%s' % str(e))
			self._failed = True
			return

		# Send the stiching request and write response to userdata
		try:
			request = SetAttachedObjectTemplateRequest(template_id = userdata.template_id,
													   pose = current_wrist_pose)
			self._srv_result = self._srv.call(self._service_topic, request)
			# The result contains the template's pose, mass, and center of mass
			userdata.template_pose = self._srv_result.template_pose
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True
			return
