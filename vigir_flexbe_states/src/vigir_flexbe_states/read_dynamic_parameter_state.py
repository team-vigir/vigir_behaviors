#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from dynamic_reconfigure.client import Client

"""
Created on 11/03/2014

@author: Philipp Schillinger
"""

class ReadDynamicParameterState(EventState):
	"""
	Reads a given trajectory controller parameter.
	"""

	LEFT_ARM_WRX = ['left_arm_traj_controller', 'l_arm_wrx']
	LEFT_ARM_WRY = ['left_arm_traj_controller', 'l_arm_wry']
	LEFT_ARM_ELX = ['left_arm_traj_controller', 'l_arm_elx']
	LEFT_ARM_ELY = ['left_arm_traj_controller', 'l_arm_ely']
	LEFT_ARM_SHX = ['left_arm_traj_controller', 'l_arm_shx']
	LEFT_ARM_SHZ = ['left_arm_traj_controller', 'l_arm_shz']

	RIGHT_ARM_WRX = ['right_arm_traj_controller', 'r_arm_wrx']
	RIGHT_ARM_WRY = ['right_arm_traj_controller', 'r_arm_wry']
	RIGHT_ARM_ELX = ['right_arm_traj_controller', 'r_arm_elx']
	RIGHT_ARM_ELY = ['right_arm_traj_controller', 'r_arm_ely']
	RIGHT_ARM_SHX = ['right_arm_traj_controller', 'r_arm_shx']
	RIGHT_ARM_SHZ = ['right_arm_traj_controller', 'r_arm_shz']
	
	def __init__(self, param):
		"""Constructor"""
		super(ReadDynamicParameterState, self).__init__(outcomes=['read', 'failed'],
												input_keys=['traj_controller'],
												output_keys=['parameter_value'])

		self._param = param
		self._failed = False

		self._clients = {}
		self._waiting_for_response = []
		self._parameter_value_list = []


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		value_offset = 0
		for i in range(len(self._clients.keys())):
			if self._waiting_for_response[i]:

				param_dict = self._clients.values()[i].get_configuration(0.1)
				if param_dict is not None:
					self._waiting_for_response[i] = False

					value_list = []
					for j in range(len(self._param.values()[i])):
						value_list.append(param_dict[self._param.values()[i][j]])

					self._parameter_value_list[value_offset:value_offset+len(value_list)] = value_list

			value_offset += len(self._param.values()[i])

		if all(not waiting for waiting in self._waiting_for_response):
			userdata.parameter_value = self._parameter_value_list
			return 'read'


	def on_enter(self, userdata):
		self._clients = {}
		self._waiting_for_response = [True] * len(self._param.keys())
		self._parameter_value_list = [None] * sum(map(len, self._param.values()))

		try:
			for server in self._param.keys():
				self._clients[server] = Client("/trajectory_controllers/" + userdata.traj_controller[0] + "/" + server + "/" + userdata.traj_controller[1])
		except Exception as e:
			Logger.logwarn('Was unable to reach parameter server:\n%s' % str(e))
			self._failed = True
