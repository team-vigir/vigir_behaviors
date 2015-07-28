#!/usr/bin/env python

import math
import rospy
import smach

from control_msgs.msg import *
from trajectory_msgs.msg import *

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from vigir_flexbe_states.msg import HandTrajectoryState

'''
Created on 04/07/2015

@author: Philipp Schillinger
'''
class FingerConfigurationState(EventState):
	'''
	Sends the fingers to a certain configuration such as opened or closed.

	-- hand_type		string 		Type of hand (e.g. 'robotiq')
	-- configuration 	float 		Fraction of which the fingers should be closed.
									1 corresponds to completely closed, 0 to opened.

	># hand_side		string 		Which hand side the trajectory refers to (e.g. 'left')

	<= done 						The trajectory was successfully executed.
	<= failed 						Failed to execute trajectory.

	'''

	def __init__(self, hand_type, configuration):
		'''Constructor'''
		super(FingerConfigurationState, self).__init__(outcomes=['done', 'failed'],
													input_keys=['hand_side'])

		self._joint_names = dict()
		self._joint_names['left'] = dict()
		self._joint_names['left']['robotiq'] = ['left_f0_j1', 'left_f1_j0', 'left_f1_j1', 'left_f2_j1']
		self._joint_names['left']['vt_hand'] = ['l_f0_j0', 'l_f1_j0']

		self._joint_names['right'] = dict()
		self._joint_names['right']['robotiq'] = ['right_f0_j1', 'right_f1_j0', 'right_f1_j1', 'right_f2_j1']
		self._joint_names['right']['vt_hand'] = ['r_f0_j0', 'r_f1_j0']

		self._joint_values = dict()
		self._joint_values['robotiq'] = map(lambda x: x * configuration, [1.22, 0.08, 1.13, 1.13]) # 2nd value is the scissor joint
		self._joint_values['vt_hand'] = map(lambda x: x * configuration, [2.685, 2.685])

		self._execution_state = HandTrajectoryState(hand_type)
		self._internal_userdata = smach.UserData()

		if not hand_type in self._joint_names['left'].keys():
			raise Exception("Unspecified hand type %s, only have %s" % (hand_type, str(self._joint_names['left'].keys())))
		self._hand_type = hand_type

		self._configuration = self._joint_values[hand_type]

		self._print = True
		self._failed = False

	
	def execute(self, userdata):
		'''During execution of the state, keep checking for action servers response'''

		if self._failed:
			return 'failed'

		self._execution_state._entering = False
		return self._execution_state.execute(self._internal_userdata)


	def on_enter(self, userdata):
		self._failed = False
		try:
			trajectory = JointTrajectory(joint_names = self._joint_names[userdata.hand_side][self._hand_type])
			trajectory.points.append(JointTrajectoryPoint(time_from_start = rospy.Duration.from_sec(0.1), positions = self._configuration))

			self._internal_userdata.finger_trajectory = trajectory
			self._internal_userdata.merge(userdata, userdata.keys(), dict())

			self._execution_state.on_enter(self._internal_userdata)
		except Exception as e:
			Logger.logwarn('Unable to execute finger configuration')
			rospy.logwarn(str(e))
			self._failed = True
