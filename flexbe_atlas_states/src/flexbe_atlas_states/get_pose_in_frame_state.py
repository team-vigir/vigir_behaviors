#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from vigir_perception_msgs.srv import *

'''
Created on 05/10/2015

@author: Spyros Maniatopoulos
'''
class GetPoseInFrameState(EventState):
	'''
	Transforms the given pose from its current frame to a target frame.

	-- target_frame string			The frame to which the pose will be transformed
									For example, 'utorso', 'world', etc.

	># pose_in 		PoseStamped 	The pose to be transformed to a target frame
									Its current frame is included in the message

	#> pose_out		PoseStamped		The same pose but in the target ref frame

	<= done 		The pose was transformed successfully.
	<= failed 		Failed to transform pose to target frame.

	'''

	def __init__(self, target_frame):
		'''Constructor'''
		super(GetPoseInFrameState, self).__init__(outcomes = ['done', 'failed'],
												  input_keys = ['pose_in'],
												  output_keys = ['pose_out'])

		self._target_frame = target_frame

		# Set up service for requesting the transformation
		self._service_topic = '/worldmodel_main/get_pose_in_frame_service'
		self._srv = ProxyServiceCaller({self._service_topic: GetPoseInFrame})
		
		self._srv_result = None

		self._failed = False
		self._done = False


	def execute(self, userdata):

		if self._failed:
			return 'failed'
		if self._done:
			return 'done'

		error = self._srv_result.error_code # message is empty if success

		if error:
			Logger.logwarn('Pose transformation failed because: %s' % error)
			self._failed = True
			return 'failed'
		else:
			userdata.pose_out = self._srv_result.transformed_pose
			return 'done'


	def on_enter(self, userdata):

		self._failed = False
		self._done = False

		# Send the transformation request
		try:
			Logger.loginfo('Requesting transformation from frame %s to %s' % 
						(userdata.pose_in.header.frame_id, self._target_frame))
			request = GetPoseInFrameRequest(pose = userdata.pose_in,
											target_frame = self._target_frame)
			self._srv_result = self._srv.call(self._service_topic, request)
		except Exception as e:
			Logger.logwarn('Failed to send service request:\n%s' % str(e))
			self._failed = True
			return
