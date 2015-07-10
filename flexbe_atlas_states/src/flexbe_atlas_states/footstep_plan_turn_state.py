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
class FootstepPlanTurnState(EventState):
	'''
	Implements a state where the robot plans a motion to turn in place, e.g. 90 degree to the left.
	Please note that the angle is only approximate, actual distance depends on exact step alignment.

	-- direction 		int 	One of the class constants to specify a turning direction.

	># angle 			float 	Angle to turn, given in degrees.

	#> plan_header 		Header  Plan to perform the turning motion.

	<= planned                 	Successfully created a plan.
	<= failed                  	Failed to create a plan.

	'''
	TURN_LEFT = 5      # PatternParameters.ROTATE_LEFT
	TURN_RIGHT = 6     # PatternParameters.ROTATE_RIGHT

	def __init__(self, direction):
		'''
		Constructor
		'''
		super(FootstepPlanTurnState, self).__init__(outcomes=['planned', 'failed'],
													input_keys=['angle'],
													output_keys=['plan_header'])

		if not rospy.has_param("behavior/turn_degrees_per_step"):
			Logger.logerr("Need to specify parameter behavior/turn_degrees_per_step at the parameter server")
			return

		self._degrees_per_step = rospy.get_param("behavior/turn_degrees_per_step")
		
		self._action_topic = '/vigir/footstep_manager/step_plan_request'

		self._client = ProxyActionClient({self._action_topic: StepPlanRequestAction})

		self._failed = False
		self._direction = direction
		
		
	def execute(self, userdata):
		if self._failed:
			userdata.plan_header = None
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.status.warning != ErrorStatus.NO_WARNING:
				Logger.logwarn('Planning footsteps warning:\n%s' % result.status.warning_msg)

			if result.status.error == ErrorStatus.NO_ERROR:
				userdata.plan_header = result.step_plan.header
				return 'planned'
			else:
				userdata.plan_header = None # as recommended: dont send out incomplete plan
				Logger.logerr('Planning footsteps failed:\n%s' % result.status.error_msg)
				return 'failed'
		

	def on_enter(self, userdata):
		# Create footstep planner request
		pattern_parameters = PatternParameters()
		pattern_parameters.mode = self._direction
		pattern_parameters.turn_angle = math.radians(self._degrees_per_step)
		pattern_parameters.close_step = True
		pattern_parameters.steps = int(round(userdata.angle / self._degrees_per_step))

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
		



