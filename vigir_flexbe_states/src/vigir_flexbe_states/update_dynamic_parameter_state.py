#!/usr/bin/env python

from flexbe_core import EventState
import rospy

from dynamic_reconfigure.client import Client

"""
Created on 11/03/2014

@author: Philipp Schillinger
"""

class UpdateDynamicParameterState(EventState):
	"""
	Updates a given trajectory controller parameter.
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
		super(UpdateDynamicParameterState, self).__init__(outcomes=['updated', 'failed'],
												input_keys=['traj_controller', 'parameter_value'])

		self._param = param
		self._failed = False

		self._clients = {}


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		value_offset = 0
		value_list = userdata.parameter_value
		for i in range(len(self._clients.keys())):

			param_dict = {}
			for j in range(len(self._param.values()[i])):
				param_dict[self._param.values()[i][j]] = value_list[value_offset + j]

			try:
				self._clients.values()[i].update_configuration(param_dict)
			except Exception as e:
				Logger.logwarn('Was unable to update parameter:\n%s' % str(e))
				return 'failed'

			value_offset += len(self._param.values()[i])

		return 'updated'


	def on_enter(self, userdata):
		self._clients = {}

		try:
			for server in self._param.keys():
				self._clients[server] = Client("/trajectory_controllers/" + userdata.traj_controller[0] + "/" + server + "/" + userdata.traj_controller[1])
		except Exception as e:
			Logger.logwarn('Was unable to reach parameter server:\n%s' % str(e))
			self._failed = True
