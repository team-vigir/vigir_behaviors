#!/usr/bin/env python

import actionlib
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from vigir_humanoid_control_msgs.msg import *

'''
Created on 22.10.2014

@author: Jochen Mueck
'''

class ChangeControlModeActionState(EventState):
	'''
	Implements a state where the robot changes its control mode using the action.

	-- target_mode 	string	The desired control mode to change to (e.g. "stand", "manipulate", etc.).
							The state's class variables can also be used (e.g. ChangeControlModeActionState.STAND).

	<= changed				Indicates successful transition to the desired control mode.
	<= failed				Indicates failure to either send the control mode change request
							or a failure to change the control mode on the Action server's side.

	'''

	NONE = "none" 
	FREEZE = "freeze" 
	STAND_PREP = "stand_prep" 
	STAND = "stand" 
	WALK = "walk" 
	STEP = "step" 
	MANIPULATE = "manipulate" 
	IMPEDANCE_STIFF = "manipulate_stiff_impedance"
	IMPEDANCE_COMPLIANT = "manipulate_compliant_impedance"
	IMPEDANCE_OBSERVER = "manipulate_observer_impedance"
	USER = "user" 
	CALIBRATE = "calibrate" 
	SOFT_STOP = "soft_stop" 
	STAND_MANIPULATE = "stand_manipulate" 
	WALK_MANIPULATE = "walk_manipulate" 
	STEP_MANIPULATE = "step_manipulate" 

	def __init__(self, target_mode):
		'''
		Constructor
		'''
		super(ChangeControlModeActionState, self).__init__(outcomes=['changed','failed'])

		if not rospy.has_param("behavior/mode_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/mode_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/mode_controllers_name")

		self._action_topic = "/" + controller_namespace + "/control_mode_controller/change_control_mode"

		self._target_mode = target_mode

		self._client = ProxyActionClient({self._action_topic: ChangeControlModeAction})

		self._failed = False


	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			# result.status == 1 means SUCCESS
			if result is None or result.result.status != 1 or self._target_mode != result.result.current_control_mode:
				rospy.logwarn('Was unable to change the control mode to %s' % str(self._target_mode))
				self._failed = True
				return 'failed'
			else:
				return 'changed'


	def on_enter(self, userdata):

		self._failed = False

		action_goal = ChangeControlModeGoal(mode_request = self._target_mode)

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send change control mode request')
			rospy.logwarn(str(e))
			self._failed = True

