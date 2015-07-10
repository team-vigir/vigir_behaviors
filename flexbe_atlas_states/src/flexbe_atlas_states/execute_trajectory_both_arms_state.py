#!/usr/bin/env python

import math
import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

'''
Created on 02/18/2015

@author: Spyros Maniatopoulos
'''
class ExecuteTrajectoryBothArmsState(EventState):
	'''
	Executes trajectory(ies) passed from userdata.

	># trajectories JointTrajectory{}	A dictionary where the keys are ['left_arm', 'right_arm'] and each has a trajectory as the value.

	<= done 							The trajectories were successfully executed.
	<= failed 							Failed to load trajectory.

	'''

	def __init__(self, controllers = ["left_arm_traj_controller", "right_arm_traj_controller"]):
		'''Constructor'''
		
		super(ExecuteTrajectoryBothArmsState, self).__init__(outcomes = ['done', 'failed'],
															 input_keys = ['trajectories'])

		self._controllers = controllers
		self._controllers = ["left_arm_traj_controller", "right_arm_traj_controller"] if not(controllers) else controllers

		self._client = ProxyActionClient()
		
		self._client_topics = dict()
		self._active_topics = list()
		
		for controller in self._controllers:
			if "left_arm" in controller:
				action_topic_left  = "/trajectory_controllers/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_left, FollowJointTrajectoryAction)
				self._client_topics['left_arm'] = action_topic_left
			elif "right_arm" in controller:
				action_topic_right = "/trajectory_controllers/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_right, FollowJointTrajectoryAction)
				self._client_topics['right_arm'] = action_topic_right
			else:
				Logger.logwarn('The controller is neither a left nor a right arm trajectory controller!? %s' % str(controller))

		self._failed = False


	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''

		if self._failed:
			return 'failed'

		results_ready = [self._client.has_result(topic) for topic in self._active_topics]

		if all(results_ready):

			results = [self._client.get_result(topic) for topic in self._active_topics]

			if any([r == None for r in results]):
				Logger.logwarn('One of the action calls returned None as result.')
				self._failed = True
				return 'failed'

			if all(r.error_code is FollowJointTrajectoryResult.SUCCESSFUL for r in results):
				return 'done'
			else:
				for r in results:
					Logger.logwarn('Joint trajectory request result: (%d) %s' % (r.error_code, r.error_string))
				Logger.logwarn('One of the requests failed to execute.')
				
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, write trajectory goal(s) to action server(s)'''

		self._failed = False

		Logger.loginfo('Received trajectory for %s arm(s).' % userdata.trajectories.keys())
		Logger.loginfo('Using %s' % str(self._controllers))

		# In execute, only check clients that have sent goals/trajectories
		self._active_topics = list()				

		# Setup client and send goal to server for execution
		try:
			for arm, traj in userdata.trajectories.iteritems():

				# Create goal message
				action_goal = FollowJointTrajectoryGoal()
				action_goal.trajectory = traj
				action_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

				topic = self._client_topics[arm]
				self._client.send_goal(topic, action_goal)
				self._active_topics.append(topic)
				
				Logger.loginfo('Goal message sent for %s trajectory' % str(arm))
	
		except Exception as e:
			Logger.logwarn('Failed to send follow joint trajectory action goal(s):\n%s' % str(e))
			self._failed = True
