#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import Image

'''
Created on 03/04/2015

@author: Philipp Schillinger
'''
class GetCameraImageState(EventState):
	'''
	Grabs the most recent camera image.

	#> camera_img 	Image 	The current color image of the left camera.

	<= done 				Image data is available.

	'''

	def __init__(self):
		'''Constructor'''
		super(GetCameraImageState, self).__init__(outcomes = ['done'],
														output_keys = ['camera_img'])

		self._img_topic = "/multisense/camera/left/image_rect_color"
		self._sub = ProxySubscriberCached({self._img_topic: Image})


	def execute(self, userdata):

		if self._sub.has_msg(self._img_topic):
			userdata.camera_img = self._sub.get_last_msg(self._img_topic)
			return 'done'


	def on_enter(self, userdata):
		pass