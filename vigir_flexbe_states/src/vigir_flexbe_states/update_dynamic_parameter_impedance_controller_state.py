#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from dynamic_reconfigure.client import Client

"""
Created on 03/05/2015

@author: Moritz Schappler
"""

class UpdateDynamicParameterImpedanceControllerState(EventState):
	"""
	Updates a given impedance controller parameter via dynamic reconfigure
	
	-- controller_chain 			List()	List of names of the chains of the controllers to be set: "left_arm", "right_arm", ...
	># parameter_keys		 		dict where keys are elements are elements of controller_chain and element is a list of parameter keys for dynamic reconfigure
	># parameter_values 			dict where keys are elements are elements of controller_chain and element is a list of parameter values for dynamic reconfigure

	<= updated 						Successfully changed dynamic reconfigure parameter.
	<= failed 						error in the update.

	"""

	
	def __init__(self, controller_chain):
		"""Constructor"""
		super(UpdateDynamicParameterImpedanceControllerState, self).__init__(outcomes=['updated', 'failed'], 
															input_keys=['parameter_keys', 'parameter_values'])

		self._controller_chain = controller_chain
		
		self._failed = False


	def execute(self, userdata):
		try:
			for entry in self._pending_clients:
	
				client = entry['client']
				param_dict = entry['parameter_dict']
				Logger.loginfo( 'UpdateDynamicParameterImpedanceControllerState: Trying client %s with parameters %s' % (client.name, str(param_dict)) )

				if param_dict is not None:
	
					client.update_configuration(param_dict)
					for key in param_dict.keys():
						Logger.loginfo("%s: %s new value: %f" % (str(client.name), key, param_dict[key]))
	
					self._pending_clients.remove(entry)
	
			if len(self._pending_clients) is 0:
				return 'updated'
		except Exception as e:
			Logger.logwarn('UpdateDynamicParameterImpedanceControllerState: Was unable to set parameters:\n%s' % str(e))
			self._failed = True
			
	def on_enter(self, userdata):
		try:
			self._parameter_keys = userdata.parameter_keys
			self._parameter_values = userdata.parameter_values
			
			if len(self._parameter_keys) != len(self._parameter_values):
				Logger.logwarn('UpdateDynamicParameterImpedanceControllerState: Parameter_keys has %d entries, but parameter_values has %d entries!' % (len(self._parameter_keys), len(self._parameter_values)) )
			
			Logger.loginfo( 'UpdateDynamicParameterImpedanceControllerState: Trying to set %d key-value paris for %s' % (len(self._parameter_keys), str(self._controller_chain)) )
			
			# dictionary, where the keys are the elements of controller_chain and the values are dicts with key-value-pairs for parameters
			self._parameter_dicts = dict()
			# dictionary, where the keys are the elements of controller_chain and the values are one client each
			self._clients = dict()
			
			# set clients and parameter dictionaries based on the controllers
			for i in range(len(self._controller_chain)):
				chain = self._controller_chain[i]
				# set controller name based on joint chain
				if chain == 'left_arm':
					controller_name = 'left_arm_joint_impedance_controller'
				elif chain == 'right_arm':
					controller_name = 'right_arm_joint_impedance_controller'
				else:
					Logger.logwarn('Chain %s not defined' % str(self._controller_chain[i]))
					return 'failed'
	
				self._clients[chain] = Client("/robot_controllers/"+controller_name+"/dyn_rec/"+self._controller_chain[i])
		
				# set parameter dictionaries
				self._parameter_dicts[chain] = dict()
				for j in range(len(self._parameter_keys[chain])):
					self._parameter_dicts[chain][self._parameter_keys[chain][j]] = self._parameter_values[chain][j]
			
			# List of pending clients will be filled with clients first and then emptied if clients have been processed
			self._pending_clients = [] 
	
			# add all clients and their parameters to the list of pending clients
			for i in range(len(self._controller_chain)):
				chain = self._controller_chain[i]
				self._pending_clients.append({'client': self._clients[chain], 'parameter_dict': self._parameter_dicts[chain]})
				Logger.loginfo( "UpdateDynamicParameterImpedanceControllerState:added pending client %s with %s" % (chain, str(self._parameter_dicts[chain])) )
		except Exception as e:
			Logger.logwarn('UpdateDynamicParameterImpedanceControllerState: Was unable to reach parameter server:\n%s' % str(e))
			self._failed = True
