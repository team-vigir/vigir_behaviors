#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flor_control_msgs.msg._FlorControlMode import FlorControlMode

'''
Created on 10/28/2014

@author: Philipp Schillinger
'''

class CheckCurrentControlModeState(EventState):
	'''
	Implements a state where the robot checks its current control mode.

	-- target_mode 	enum	The control mode to check for being the current one (e.g. 3 for stand, 6 for manipulate).
							The state's class variables can also be used (e.g. CheckCurrentControlModeState.STAND).
	-- wait 		bool	Whether to check once and return (False), or wait for the control mode to change (True).

	#> control_mode enum	The current control mode when the state returned.

	<= correct				Indicates that the current control mode is the target/expected one.
	<= incorrect			Indicates that the current control mode is not the target/expected one.

	'''

	NONE = 0
	FREEZE = 1
	STAND_PREP = 2
	STAND = 3
	STAND_MANIPULATE = 3
	WALK = 4
	STEP = 5
	MANIPULATE = 6
	USER = 7
	CALIBRATE = 8
	SOFT_STOP = 9


	def __init__(self, target_mode, wait = False):
		'''
		Constructor
		'''
		super(CheckCurrentControlModeState, self).__init__(outcomes=['correct', 'incorrect'],
														   output_keys=['control_mode'])
		
		self._status_topic = '/flor/controller/mode'
		
		self._sub = ProxySubscriberCached({self._status_topic: FlorControlMode})
		
		self._sub.make_persistant(self._status_topic)
		
		self._target_mode = target_mode
		self._wait = wait
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._sub.has_msg(self._status_topic):
			msg = self._sub.get_last_msg(self._status_topic)
			userdata.control_mode = msg.bdi_current_behavior
			if msg.bdi_current_behavior == self._target_mode:
				return 'correct'
			elif not self._wait:
				return 'incorrect'
		
	def on_enter(self, userdata):
		if self._wait:
			Logger.loghint("Waiting for %s" % str(self._target_mode))
