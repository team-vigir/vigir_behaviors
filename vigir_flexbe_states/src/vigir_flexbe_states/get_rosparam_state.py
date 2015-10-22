#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

"""
Created on 10/22/2015

@author: Achim Stein
"""

class GetRosParamState(EventState):
	"""
	Set a parameter on the ROS parameter server

	-- parameter 		string 	 	The parameter to be fetched
	
	#> value 			string 		The fetch parameter value
		
	"""
	
	def __init__(self, parameter):
		"""Constructor"""
		super(GetRosParamState, self).__init__(outcomes=['done', 'failed'],
										       output_keys = ['value'])

		self._failed = False
		self._parameter = parameter;		


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
			if not rospy.has_param(self._parameter):
				self._failed = True
			else:
				userdata.value = rospy.get_param(self._parameter)
		except Exception as e:
			Logger.logwarn('Was unable to set parameter %s to %s:\n%s' % (self._parameter, self._value, str(e)))
			self._failed = True
