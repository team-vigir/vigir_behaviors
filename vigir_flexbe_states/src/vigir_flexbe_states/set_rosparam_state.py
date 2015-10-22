#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

"""
Created on 10/22/2015

@author: Achim Stein
"""

class SetRosParamState(EventState):
	"""
	Set a parameter on the ROS parameter server

	-- parameter 		string 	 	The parameter to be set
	-- value			string		String representation of the value to be set
	
	"""
	
	def __init__(self, parameter, value):
		"""Constructor"""
		super(SetRosParamState, self).__init__(outcomes=['done', 'failed'])

		self._failed = False
		self._parameter = parameter;
		self._value = value;


	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''
		
		if self._failed:
			return 'failed'
		else:
			return 'done'
	
	def on_enter(self, userdata):
		"""Set the parameter on the server"""

		self._failed = False
		try:
			rospy.set_param(self._parameter, self._value)
		except Exception as e:
			Logger.logwarn('Was unable to set parameter %s to %s:\n%s' % (self._parameter, self._value, str(e)))
			self._failed = True
