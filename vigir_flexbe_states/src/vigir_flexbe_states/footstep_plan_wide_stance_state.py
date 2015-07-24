#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from vigir_footstep_planning_msgs.msg import *
from std_msgs.msg import String, Header

'''
Created on 02/24/2015

@author: Philipp Schillinger and Spyros Maniatopoulos
'''
class FootstepPlanWideStanceState(EventState):
	'''
	Implements a state where the robot plans to move into wide stance.

	#> plan_header    Header    Plan to go to the desired configuration.

	<= planned                 	Successfully created a plan.
	<= failed                  	Failed to create a plan.

	'''
	def __init__(self):
		super(FootstepPlanWideStanceState, self).__init__(outcomes=['planned','failed'],
														  output_keys=['plan_header'])

		self._action_topic = '/vigir/footstep_manager/step_plan_request'

		self._client = ProxyActionClient({self._action_topic: StepPlanRequestAction})

		self._failed = False
		self._done = False
		

	def execute(self, userdata):
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
				self._done = True
				return 'planned'
			else:
				userdata.plan_header = None # as recommended: dont send out incomplete plan
				Logger.logerr('Planning footsteps failed:\n%s' % result.status.error_msg)
				self._failed = True
				return 'failed'
		
	
	def on_enter(self, userdata):
		''' 
		Sending plan request with wide stance mode
		'''
		self._failed = False
		self._done = False

		pattern_parameters = PatternParameters()
		pattern_parameters.steps = 4
		pattern_parameters.mode = PatternParameters.WIDE_STANCE
		pattern_parameters.step_distance_forward = 0.0
		pattern_parameters.step_distance_sideward = 0.07
		pattern_parameters.turn_angle = math.radians(20.0)
		pattern_parameters.close_step = True # feels wrong...
		pattern_parameters.extra_seperation = False

		request = StepPlanRequest()
		request.parameter_set_name = String('drc_step_no_collision')
		request.header = Header(frame_id = '/world', stamp = rospy.Time.now())
		request.planning_mode = StepPlanRequest.PLANNING_MODE_PATTERN
		request.pattern_parameters = pattern_parameters

		action_goal = StepPlanRequestGoal(plan_request = request)

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Was unable to create footstep pattern for wide stance:\n%s' % str(e))
			self._failed = True
			
		
		 
