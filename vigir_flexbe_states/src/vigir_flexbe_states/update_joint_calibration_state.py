#!/usr/bin/env python

import rospy
from copy import copy

from flexbe_core import EventState, Logger

from dynamic_reconfigure.client import Client

"""
Created on 11/03/2014

@author: Philipp Schillinger
"""

class UpdateJointCalibrationState(EventState):
	"""
	Updates the calibration offsets for the given joints.

	-- joint_names 		string[]	Names of the joints to be calibrated. Order should match the given offset values.

	># joint_offsets 	float[] 	Determined joint offset to be calibrated.

	<= updated 						Successfully updated the calibration of all joints.
	<= failed 						Failed to contact the calibration parameter server.

	"""
	
	def __init__(self, joint_names):
		"""Constructor"""
		super(UpdateJointCalibrationState, self).__init__(outcomes=['updated', 'failed'],
												input_keys=['joint_offsets'])

		self._joint_names = joint_names
		self._failed = False

		self._clients = list()
		for i in range(len(self._joint_names)):
			self._clients.append(Client("/atlas_controller/calibration/" + self._joint_names[i]))

		self._pending_clients = list()

	def execute(self, userdata):
		if self._failed:
			return 'failed'

		for entry in self._pending_clients:

			client = entry['client']
			offset_update = entry['offset_update']

			param_dict = client.get_configuration(0.1)
			if param_dict is not None:
				
				current_offset = param_dict['offset']
				new_offset = current_offset + offset_update
				client.update_configuration({'offset': new_offset})

				Logger.loginfo("%s: %.3f --> %.3f" % (str(client.name), current_offset, new_offset))

				self._pending_clients.remove(entry)

		if len(self._pending_clients) is 0:
			return 'updated'


	def on_enter(self, userdata):
		
		self._failed = False

		self._pending_clients = list()

		try:
			for i in range(len(self._clients)):
				self._pending_clients.append({'client': self._clients[i], 'offset_update': userdata.joint_offsets[i]})

		except Exception as e:
			Logger.logwarn('Was unable to reach parameter server:\n%s' % str(e))
			self._failed = True
