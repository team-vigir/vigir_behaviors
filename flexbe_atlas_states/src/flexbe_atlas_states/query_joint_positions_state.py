#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from control_msgs.srv import *

'''
Created on 05/19/2015

@author: Spyros Maniatopoulos
'''

class QueryJointPositionsState(EventState):
	'''
	Retrieves the current positions of the joints of the specified planning group.

	-- side 			string		One of left, right
	-- controller 		string		Specifify the trajectory controller suffix
									Example: 'traj_hybrid_controller'

	#> joint_config 	dict		Trajectory joint positions at current time.
									joint_names string[] : joint_values[]

	<= retrieved 					Successfully determined the current joint values.
	<= failed 						Failed to send service call or got no result.

	'''
	
	TRAJ_CONTROLLER = 'traj_controller'
	HYBRID_CONTROLLER = 'traj_hybrid_controller'

	def __init__(self, side, controller = 'traj_controller'):
		'''
		Constructor
		'''
		super(QueryJointPositionsState, self).__init__(outcomes = ['retrieved', 'failed'],
												   	   output_keys = ['joint_config'])
		
		self._srv_topic = '/joint_controllers/' + side + '_arm_' + \
						  controller + '/query_state'
		
		self._srv = ProxyServiceCaller({self._srv_topic: QueryTrajectoryState})
				
		self._srv_result = None
		self._failed = False

		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'
		else:
			return 'retrieved'
		
	
	def on_enter(self, userdata):

		self._failed = False

		try:
			self._srv_result = self._srv.call(
				self._srv_topic, 
				QueryTrajectoryStateRequest(time = rospy.Time.now()))

			joint_names  = self._srv_result.name
			joint_values = list(self._srv_result.position) # cast tuple to list
			
			userdata.joint_config = {'joint_names' : joint_names,
										'joint_values': joint_values}
		except Exception as e:
			Logger.logwarn('Could not retrieve current joint positions!\n%s' % str(e))
			self._failed = True
