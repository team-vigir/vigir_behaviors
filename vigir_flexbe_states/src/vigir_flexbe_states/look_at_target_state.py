#!/usr/bin/env python

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from vigir_planning_msgs.msg import HeadControlCommand

'''
Created on 05/23/2015

@author: Philipp Schillinger
'''

class LookAtTargetState(EventState):
	'''
	A state that can look at any link target, e.g. one of the hands or a template.
	
	># frame 	string 		Frame to be tracked, pass None to look straight.

	<= done 				Command has been sent to head control.
	
	'''
	
	def __init__(self):
		'''Constructor'''
		super(LookAtTargetState, self).__init__(outcomes=['done'],
												input_keys=['frame'])
		
		self._head_control_topic = '/thor_mang/head_control_mode'

		self._pub = ProxyPublisher({self._head_control_topic: HeadControlCommand})

		
	def execute(self, userdata):
		'''Execute this state'''
		
		return 'done'
		

	
	def on_enter(self, userdata):
		'''Entering the state.'''
		command = HeadControlCommand()

		if userdata.frame is not None:
			command.motion_type = HeadControlCommand.TRACK_FRAME
			command.tracking_frame = userdata.frame
		else:
			command.motion_type = HeadControlCommand.LOOK_STRAIGHT

		self._pub.publish(self._head_control_topic, command)


		
