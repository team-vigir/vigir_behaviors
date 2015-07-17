#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from vigir_planning_msgs.srv import *

'''
Created on 04.12.2013

@author: Philipp Schillinger
'''

class CurrentJointPositionsState(EventState):
	'''
	Retrieves the current positions of the joints of the specified planning group.

	-- planning_group 	string 		Name of the planning group.

	#> joint_positions 	float[]		Current joint values. Length of the list depends on the number of joints of the planning group.

	<= retrieved 					Successfully determined the current joint values.
	<= failed 						Failed to send service call or got no result.

	'''
	

	def __init__(self, planning_group):
		'''
		Constructor
		'''
		super(CurrentJointPositionsState, self).__init__(outcomes=['retrieved', 'failed'],
												   		 output_keys=['joint_positions'])
		
		self._srv_topic = '/flor/planning/upper_body/get_group_state'
		
		self._srv = ProxyServiceCaller({self._srv_topic: GetCurrentPlanningGroupState})
		
		self._planning_group = planning_group
		
		self._srv_result = None
		self._failed = False
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'

		if self._srv_result is not None:
			userdata.joint_positions = self._srv_result.position
			
			if len(self._srv_result.position) == 0:
				Logger.logwarn("Lookup of joint positions of group %s failed" % self._planning_group)
				self._failed = True
				return 'failed'
			else:
				return 'retrieved'
		
	
	def on_enter(self, userdata):

		self._failed = False

		try:
			self._srv_result = self._srv.call(
				self._srv_topic, 
				GetCurrentPlanningGroupStateRequest(self._planning_group))
		
		except Exception as e:
			Logger.logwarn('Could not retrieve current joint positions!')
			rospy.logwarn(str(e))
			self._failed = True
		
		
	
	



