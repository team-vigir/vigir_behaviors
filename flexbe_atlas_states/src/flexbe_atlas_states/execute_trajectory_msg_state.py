#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

'''
Created on 10/31/2014

@author: Philipp Schillinger
'''
class ExecuteTrajectoryMsgState(EventState):
	'''
	Executes a given joint trajectory message.

	-- controller       	string      		One of the class constants to specify the controller which should execute the given trajectory.

	># joint_trajectory 	JointTrajectory		Trajectory to be executed, containing all required information.

	<= done 									Trajectory has successfully finished its execution.
	<= failed 									Failed to execute trajectory.

	'''

	CONTROLLER_LEFT_ARM = "left_arm_traj_controller"
	CONTROLLER_RIGHT_ARM = "right_arm_traj_controller"
	CONTROLLER_PELVIS = "pelvis_traj_controller"
	CONTROLLER_TORSO = "torso_traj_controller"
	CONTROLLER_LEFT_LEG = "left_leg_traj_controller"
	CONTROLLER_RIGHT_LEG = "right_leg_traj_controller"
	CONTROLLER_NECK = "neck_traj_controller"


	def __init__(self, controller):
		'''
		Constructor
		'''
		super(ExecuteTrajectoryMsgState, self).__init__(outcomes=['done', 'failed'],
														input_keys=['joint_trajectory'])


		if not rospy.has_param("behavior/joint_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/joint_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/joint_controllers_name")

		self._action_topic = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"

		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._failed = False


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result:
				if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
					return 'done'
				else:
					Logger.logwarn('Joint trajectory request failed to execute (%d). Reason: %s' % (result.error_code, result.error_string))
					self._failed = True
					return 'failed'
			else:
				Logger.logwarn('Wait for result returned True even though the result is %s' % str(result))
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		self._failed = False

		# create goal
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = userdata.joint_trajectory
		goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

		# execute the motion
		try: 
			self._client.send_goal(self._action_topic, goal)
		except Exception as e:
			Logger.logwarn('Was unable to execute joint trajectory:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")


