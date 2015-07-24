#!/usr/bin/env python

import rospy
import pickle
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

'''
Created on 03/04/2015

@author: Philipp Schillinger
'''
class SendToOperatorState(EventState):
	'''
	Sends the provided data to the operator using a generic serialized topic.

	># data 	object 	The data to be sent to the operator.
						You should be aware of required bandwidth consumption.

	<= done 			Data has been sent to onboard.
						This is no guarantee that it really has been received.
	<= no_connection 	Unable to send the data.

	'''

	def __init__(self):
		'''Constructor'''
		super(SendToOperatorState, self).__init__(outcomes = ['done', 'no_connection'],
														input_keys = ['data'])

		self._data_topic = "/flexbe/data_to_ocs"
		self._pub = ProxyPublisher({self._data_topic: String})

		self._failed = False


	def execute(self, userdata):
		if self._failed:
			return 'no_connection'

		return 'done'


	def on_enter(self, userdata):
		self._failed = False

		try:
			self._pub.publish(self._data_topic, String(pickle.dumps(userdata.data)))
		
		except Exception as e:
			Logger.logwarn('Failed to send data to OCS:\n%s' % str(e))
			self._failed = True