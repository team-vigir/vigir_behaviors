#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger


'''
Created on 09/02/2015

@author: Achim Stein
'''
class GetTFTransformState(EventState):
	'''
	Retrieves the transformation between two frames

	># frames 	string[] 		frames[0] = target frame, frames[1] = source frame
	
	#> transform 	PoseStamped	The current transformation between the frames

	<= done 					Transformation has been retrieved.
	<= failed 					Failed to retrieve transformation.

	'''

	def __init__(self):
		'''Constructor'''
		super(GetTFTransformState, self).__init__(outcomes = ['done', 'failed'],
												input_keys = ['frames'],
												output_keys = ['transform'])
		
		self._failed = False


	def execute(self, userdata):

		if self._failed:
			return 'failed'
		else:
			return 'done'


	def on_enter(self, userdata):
		self._failed = False
				
		listener = tf.TransformListener()
		transform = PoseStamped()
				
		if len(userdata.frames) != 2:
			self._failed = True
			Logger.logwarn('Did not receive two frames')
			return
		
		target_frame = userdata.frames[0]
		source_frame = userdata.frames[1]		
		Logger.loginfo('Getting transform for target = %s, source = %s' % (target_frame, source_frame))

		try:
			now = rospy.Time(0)
			listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(4.0))
			(trans,rot) = listener.lookupTransform(target_frame, source_frame, now)
			
			transform.pose.position.x = trans[0]
			transform.pose.position.y = trans[1]
			transform.pose.position.z = trans[2]
			transform.pose.orientation.x = rot[0]
			transform.pose.orientation.y = rot[1]
			transform.pose.orientation.z = rot[2]
			transform.pose.orientation.w = rot[3]
			
			userdata.transform = transform
			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			Logger.logwarn('Failed to get the transformation:\n%s' % str(e))			
			self._failed = True
			userdata.transform = None
			return
		