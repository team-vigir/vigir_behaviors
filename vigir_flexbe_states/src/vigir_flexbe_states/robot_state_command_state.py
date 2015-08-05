#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from vigir_control_msgs.msg import VigirRobotStateCommand

'''
Created on 05/23/2015

@author: Spyros Maniatopoulos
'''
class RobotStateCommandState(EventState):
	'''
	Publishes a robot state command message.

	-- command 	int 	Command to be sent. Use class variables (e.g. STAND).

	<= done				Successfully published the state command message.
	<= failed			Failed to publish the state command message.

	'''

	START_HYDRAULIC_PRESSURE_OFF = 4 # "fake" start (without hydraulic pressure)
	START_HYDRAULIC_PRESSURE_ON = 6	 # start normally (with hydraulic pressure)
	STOP = 8 # stop the pump
	FREEZE = 16
	STAND = 32
	STAND_PREP = 33
	CALIBRATE = 64 # BIASES
	CALIBRATE_ARMS = 128
	CALIBRATE_ARMS_FREEZE = 144

	def __init__(self, command):
		'''Constructor'''
		super(RobotStateCommandState, self).__init__(outcomes = ['done', 'failed'])

		self._topic = '/flor/controller/robot_state_command'
		self._pub = ProxyPublisher({self._topic: VigirRobotStateCommand})

		self._command = command

		self._failed = False


	def execute(self, userdata):
		if self._failed:
			return 'failed'
		else:
			return 'done'


	def on_enter(self, userdata):
		self._failed = False

		try:
			command_msg = VigirRobotStateCommand(state_command = self._command)
			self._pub.publish(self._topic, command_msg)
		
		except Exception as e:
			Logger.logwarn('Failed to publish the command message:\n%s' % str(e))
			self._failed = True
