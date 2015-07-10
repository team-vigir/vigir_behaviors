#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached


'''
Created on 05/12/2015

@author: Spyros Maniatopoulos
'''
class GetWristPoseState(EventState):
	'''
	Retrieves the current wrist pose from the corresponding ROS topic.

	># hand_side 	string 		Wrist whose pose will be returned (left, right)

	#> wrist_pose 	PoseStamped	The current pose of the left or right wrist

	<= done 					Wrist pose has been retrieved.
	<= failed 					Failed to retrieve wrist pose.

	'''

	def __init__(self):
		'''Constructor'''
		super(GetWristPoseState, self).__init__(outcomes = ['done', 'failed'],
												input_keys = ['hand_side'],
												output_keys = ['wrist_pose'])

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
		else:
			return 'done'


	def on_enter(self, userdata):

		self._failed = False

		# Read the current wrist pose and write to userdata
		try:
			wrist_pose_topic = self._wrist_pose_topics[userdata.hand_side]
			wrist_pose = self._sub.get_last_msg(wrist_pose_topic)
			userdata.wrist_pose = wrist_pose
			rospy.loginfo('Retrieved %s wrist pose: \n%s',
						  userdata.hand_side, wrist_pose)
		except Exception as e:
			Logger.logwarn('Failed to get the latest wrist pose:\n%s' % str(e))
			self._failed = True
			return
