#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from vigir_ocs_msgs.msg import OCSLogging


"""Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

class VideoLoggingState(EventState):
	"""
	A state that controls video logging.

	-- command 			boolean 	One of the available commands provided as class constants.
	-- no_video 		boolean 	Only create bag files.
	-- no_bags			boolean 	Only record video.

	># experiment_name 	string 		Unique name of the experiment.

	<= done 						Executed the specified command.

	"""
	
	START = True
	STOP  = False

	def __init__(self, command, no_video=False, no_bags=True):
		"""Constructor"""
		super(VideoLoggingState, self).__init__(outcomes=['done'],
												input_keys=['experiment_name', 'description'])

		self._topic = "/vigir_logging"
		self._pub   = ProxyPublisher({self._topic: OCSLogging})

		self._command  = command
		self._no_video = no_video
		self._no_bags  = no_bags

	def execute(self, userdata):
		"""Execute this state"""

		# nothing to check
		return 'done'
	
	def on_enter(self, userdata):
		"""Upon entering the state"""

		try:
			self._pub.publish(self._topic, OCSLogging(run=self._command, no_video=self._no_video, no_bags=self._no_bags, experiment_name=userdata.experiment_name, description=userdata.description))
		except Exception as e:
			Logger.logwarn('Could not send video logging command:\n %s' % str(e))
		
	def on_stop(self):
		"""Stop video logging upon end of execution"""
		
		try:
			self._pub.publish(self._topic, OCSLogging(run=False))
		except Exception as e:
			pass
		
