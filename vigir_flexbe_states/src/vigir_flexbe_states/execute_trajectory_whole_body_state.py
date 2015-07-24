#!/usr/bin/env python

import math
import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

'''
Created on 04/13/2015

@author: Moritz Schappler
'''
class ExecuteTrajectoryWholeBodyState(EventState):
	'''
	Executes trajectory(ies) passed from userdata.

	># trajectories JointTrajectory{}	A dictionary where the keys are ['left_arm', 'right_arm', 'left_leg', 'right_leg', 'torso'] and each has a trajectory as the value.

	<= done 							The trajectories were successfully executed.
	<= failed 							Failed to load trajectory.

	'''
	
	CONTROLLER_LEFT_ARM = "left_arm_traj_controller"
	CONTROLLER_RIGHT_ARM = "right_arm_traj_controller"
	CONTROLLER_PELVIS = "pelvis_traj_controller"
	CONTROLLER_TORSO = "torso_traj_controller"
	CONTROLLER_LEFT_LEG = "left_leg_traj_controller"
	CONTROLLER_RIGHT_LEG = "right_leg_traj_controller"
	CONTROLLER_NECK = "neck_traj_controller"
	

	def __init__(self, controllers = ["left_arm_traj_controller", "right_arm_traj_controller", "torso_traj_controller", "left_leg_traj_controller", "right_leg_traj_controller"]):
		'''Constructor'''
		
		super(ExecuteTrajectoryWholeBodyState, self).__init__(outcomes = ['done', 'failed'],
															 input_keys = ['trajectories'])

		if not rospy.has_param("behavior/joint_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/joint_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/joint_controllers_name")

		self._controllers = controllers
		self._controllers = ["torso_traj_controller", "left_leg_traj_controller", "right_leg_traj_controller", "left_arm_traj_controller", "right_arm_traj_controller"] if not(controllers) else controllers

		self._client = ProxyActionClient()
		
		self._client_topics = dict()
		self._active_topics = list()
		
		for controller in self._controllers:
			if "left_arm" in controller:
				action_topic_left  = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_left, FollowJointTrajectoryAction)
				self._client_topics['left_arm'] = action_topic_left
			elif "right_arm" in controller:
				action_topic_right = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_right, FollowJointTrajectoryAction)
				self._client_topics['right_arm'] = action_topic_right
			elif "left_leg" in controller:
				action_topic_left  = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_left, FollowJointTrajectoryAction)
				self._client_topics['left_leg'] = action_topic_left
			elif "right_leg" in controller:
				action_topic_right = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_right, FollowJointTrajectoryAction)
				self._client_topics['right_leg'] = action_topic_right
			elif "torso" in controller:
				action_topic_right = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"
				self._client.setupClient(action_topic_right, FollowJointTrajectoryAction)
				self._client_topics['torso'] = action_topic_right
			else:
				Logger.logwarn('ExecuteTrajectoryWholeBodyState:The controller is neither an arm, leg or torso trajectory controller!? %s' % str(controller))
		print self._client_topics
		print self._client
		self._failed = False


	def execute(self, userdata):
		'''Code to be executed while SM is in this state.'''

		if self._failed:
			return 'failed'
		results_ready = [self._client.has_result(topic) for topic in self._active_topics]
		if all(results_ready):
			results = [self._client.get_result(topic) for topic in self._active_topics]
			if all(r.error_code is FollowJointTrajectoryResult.SUCCESSFUL for r in results):
				return 'done'
			else:
				for r in results:
					Logger.logwarn('ExecuteTrajectoryWholeBodyState: Joint trajectory request result: (%d) %s' % (r.error_code, r.error_string))
				Logger.logwarn('ExecuteTrajectoryWholeBodyState: One of the requests failed to execute.')
				
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, write trajectory goal(s) to action server(s)'''

		self._failed = False
		Logger.loginfo('ExecuteTrajectoryWholeBodyState: Received trajectory for %s.' % userdata.trajectories.keys())

		# In execute, only check clients that have sent goals/trajectories
		self._active_topics = list()				

		# Setup client and send goal to server for execution
		try:
			for chain, traj in userdata.trajectories.iteritems():
			
				# Create goal message
				action_goal = FollowJointTrajectoryGoal()
				action_goal.trajectory = traj
				action_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
				
				topic = self._client_topics[chain]
				self._client.send_goal(topic, action_goal)
				self._active_topics.append(topic)
				
				Logger.loginfo('ExecuteTrajectoryWholeBodyState: Goal message sent for %s (topic %s)' % (str(chain), str(topic)))
			
		except Exception as e:
			Logger.logwarn('ExecuteTrajectoryWholeBodyState: Failed to send follow joint trajectory action goal(s):\n%s' % str(e))
			self._failed = True


			


