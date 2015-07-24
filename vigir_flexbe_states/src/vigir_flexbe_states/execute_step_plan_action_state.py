#!/usr/bin/env python

import actionlib
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from vigir_footstep_planning_msgs.msg import *

'''
Created on 22.10.2014

@author: Jochen Mueck, Philipp Schillinger, and Spyros Maniatopoulos
'''

class ExecuteStepPlanActionState(EventState):
	'''
	Implements a state to execute a step plan of the footstep planner via the OBFSM.
	This state will change the control mode of the robot to STEP during execution and expects to be in STAND when entered.

	># plan_header 	Header 		Header of the footstep plan to be executed.
								Use one of the footstep planning states to calculate such a plan.

	<= finished 				Finished walking.
								Control mode will be changed back to STAND, but this can take some time.
								If you need to be in STAND for the next state, add a CheckCurrentControlModeState.
	
	<= failed 					Failed to completely execute the plan. Plan might have been executed partially.

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(ExecuteStepPlanActionState , self).__init__(outcomes=['finished','failed'],
														  input_keys=['plan_header'])

		self._action_topic = "/vigir/footstep_manager/execute_step_plan"
		self._client = ProxyActionClient({self._action_topic: ExecuteStepPlanAction})

		self._failed = False


	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if (result is not None and 
				result.status.status == FootstepExecutionStatus.REACHED_GOAL):
				return 'finished'
			else:
				if result is None:
					Logger.logwarn("Got None as result")
				else:
					Logger.logwarn("Result status %d" % result.status.status)
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, request step plan execution from OBFSM'''
		
		execution_request = StepPlan(header = userdata.plan_header)

		action_goal = ExecuteStepPlanGoal(step_plan = execution_request)
		
		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Unable to execute step plan')
			rospy.logwarn(str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")

