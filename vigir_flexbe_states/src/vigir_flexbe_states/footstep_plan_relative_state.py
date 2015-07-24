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
class FootstepPlanRelativeState(EventState):
	'''
	Implements a state where the robot plans a relative motion, e.g. 2m to the left.
	Please note that the distance is only approximate, actual distance depends on exact step alignment.

	-- direction 	int 	One of the class constants to specify a direction.

	># distance 	float 	Distance to walk, given in meters.

	#> plan_header 	Header 	The header of the plan to perform the walking.

	<= planned 				Successfully created a plan.
	<= failed 				Failed to create a plan.

	'''
	DIRECTION_LEFT = 3      # PatternParameters.STRAFE_LEFT
	DIRECTION_RIGHT = 4     # PatternParameters.STRAFE_RIGHT
	DIRECTION_FORWARD = 1   # PatternParameters.FORWARD
	DIRECTION_BACKWARD = 2  # PatternParameters.BACKARD

	def __init__(self, direction):
		'''
		Constructor
		'''
		super(FootstepPlanRelativeState, self).__init__(outcomes=['planned', 'failed'],
														input_keys=['distance'],
														output_keys=['plan_header'])

		if not rospy.has_param("behavior/step_distance_forward"):
			Logger.logerr("Need to specify parameter behavior/step_distance_forward at the parameter server")
			return
		if not rospy.has_param("behavior/step_distance_sideward"):
			Logger.logerr("Need to specify parameter behavior/step_distance_sideward at the parameter server")
			return

		self._step_distance_forward = rospy.get_param("behavior/step_distance_forward")
		self._step_distance_sideward = rospy.get_param("behavior/step_distance_sideward")
		
		self._action_topic = '/vigir/footstep_manager/step_plan_request'

		self._client = ProxyActionClient({self._action_topic: StepPlanRequestAction})

		self._done   = False
		self._failed = False
		self._direction = direction
		
		
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
				
				num_steps = len(result.step_plan.steps)
				Logger.loginfo('Received plan with %d steps' % num_steps)
				
				self._done = True
				return 'planned'
			else:
				userdata.plan_header = None # as recommended: dont send out incomplete plan
				Logger.logerr('Planning footsteps failed:\n%s' % result.status.error_msg)
				self._failed = True
				return 'failed'
		

	def on_enter(self, userdata):

		self._failed = False
		self._done = False

		# Create footstep planner request
		strafing = self._direction == PatternParameters.STRAFE_LEFT or self._direction == PatternParameters.STRAFE_RIGHT
		pattern_parameters = PatternParameters()
		pattern_parameters.mode = self._direction
		pattern_parameters.step_distance_forward = self._step_distance_forward if not strafing else 0.0 # will it ignore?
		pattern_parameters.step_distance_sideward = self._step_distance_sideward if strafing else 0.0 # will it ignore?
		pattern_parameters.close_step = True
		step_distance = pattern_parameters.step_distance_sideward if strafing else pattern_parameters.step_distance_forward
		pattern_parameters.steps = int(round(userdata.distance / step_distance))

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
		



