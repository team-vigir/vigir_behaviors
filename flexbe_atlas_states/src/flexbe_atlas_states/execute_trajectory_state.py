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
class ExecuteTrajectoryState(EventState):
	'''
	Executes a custom trajectory.

	-- controller       string      One of the class constants to specify the controller which should execute the given trajectory.
	-- joint_names      string[]    List of the joint names. Order has to match the order of the provided values.

	># joint_positions  float[][]	Trajectory to be executed, given as list of time steps where each step contains a list of target joint values.
	># time 			float[]		Relative time in seconds from starting the execution when the corresponding time step should be reached.

	<= done 						Trajectory has successfully finished its execution.
	<= failed 						Failed to execute trajectory.

	'''

	CONTROLLER_LEFT_ARM = "left_arm_traj_controller"
	CONTROLLER_RIGHT_ARM = "right_arm_traj_controller"
	CONTROLLER_PELVIS = "pelvis_traj_controller"
	CONTROLLER_TORSO = "torso_traj_controller"
	CONTROLLER_LEFT_LEG = "left_leg_traj_controller"
	CONTROLLER_RIGHT_LEG = "right_leg_traj_controller"
	CONTROLLER_NECK = "neck_traj_controller"


	def __init__(self, controller, joint_names):
		'''
		Constructor
		'''
		super(ExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed'],
														input_keys=['joint_positions', 'time'])

		if not rospy.has_param("behavior/joint_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/joint_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/joint_controllers_name")

		self._joint_names = joint_names
		self._action_topic = "/" + controller_namespace + "/" + controller + "/follow_joint_trajectory"

		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._failed = False


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
				return 'done'
			else:
				Logger.logwarn('Joint trajectory request failed to execute: (%d) %s' % (result.error_code, result.error_string))
				return 'failed'


	def on_enter(self, userdata):
		# create goal
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = JointTrajectory()
		goal.trajectory.joint_names = self._joint_names
		goal.trajectory.points = []

		for i in range(len(userdata.joint_positions)):
			point = JointTrajectoryPoint()
			point.positions = userdata.joint_positions[i]
			point.time_from_start = rospy.Duration.from_sec(userdata.time[i])
			goal.trajectory.points.append(point)

		# execute the motion
		try: 
			self._client.send_goal(self._action_topic, goal)
		except Exception as e:
			Logger.logwarn('Was unable to execute joint trajectory:\n%s' % str(e))
			self._failed = True


