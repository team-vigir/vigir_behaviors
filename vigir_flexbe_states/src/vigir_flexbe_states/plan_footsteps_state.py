#!/usr/bin/env python

import actionlib
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxyActionClient

from vigir_footstep_planning_msgs.msg import *
from std_msgs.msg import *

'''
Created on 10/22/2014

@author: Philipp Schillinger and Spyros Maniatopoulos
'''

class PlanFootstepsState(EventState):
	'''
	Creates a footstep plan to reach the desired feet pose via the OBFSM.

	-- mode 			string 		One of the available planning modes (class constants).

	># step_goal 		Feet 		Desired feet pose.

	#> plan_header 		Header 		The header of the footstep plan.

	<= planned 						Successfully created a plan.
	<= failed 						Failed to create a plan.

	'''

	MODE_STEP_NO_COLLISION = 'drc_step_no_collision'
	MODE_STEP_2D = 'drc_step_2D'
	MODE_STEP_3D = 'drc_step_3D'
	MODE_WALK = 'drc_walk'

	def __init__(self, mode):
		'''
		Constructor
		'''
		super(PlanFootstepsState, self).__init__(outcomes=['planned', 'failed'],
												 input_keys=['step_goal'],
												 output_keys=['plan_header'])

		self._action_topic = "/vigir/footstep_manager/step_plan_request"

		self._client = ProxyActionClient({self._action_topic: StepPlanRequestAction})

		self._mode = mode

		self._done   = False
		self._failed = False


	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			userdata.plan_header = None
			return 'failed'
		if self._done:
			return 'planned'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.status.warning != ErrorStatus.NO_WARNING:
				Logger.logwarn('Planning footsteps warning:\n%s' % result.status.warning_msg)

			if result.status.error == ErrorStatus.NO_ERROR:
				userdata.plan_header = result.step_plan.header

				num_steps = len(result.step_plan.steps)
				Logger.loginfo('Received plan with %d steps' % num_steps)
				
				self._done = True
				return 'planned'
			else:
				userdata.plan_header = None
				Logger.logerr('Planning footsteps failed:\n%s' % result.status.error_msg)
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, send the footstep plan request.'''

		self._failed = False
		self._done = False

		# Create request msg and action goal
		request = StepPlanRequest()
		request.header = userdata.step_goal.header
		request.max_planning_time = 10.0
		request.parameter_set_name = String(self._mode)
		
		action_goal = StepPlanRequestGoal(plan_request = request)

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send footstep plan request:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		'''Destructor'''
		
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")
