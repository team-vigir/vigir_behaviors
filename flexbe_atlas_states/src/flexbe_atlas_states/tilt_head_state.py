#!/usr/bin/env python

import math

import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

'''
Created on 05/13/2015

@author: Spyros Maniatopoulos
'''
class TiltHeadState(EventState):
	'''
	Executes a given joint trajectory message.

	-- desired_tilt 	float 	Degrees (positive is down, negative is up).
						Class variables are available (UP, DOWN, STRAIGHT).

	<= done 			The head was successfully tilted.
	<= failed 			Failed to execute neck trajectory.

	'''

	UP_40 	 = -40.0 # max -40 degrees
	UP_20 	 = -20.0
	STRAIGHT = +0.00
	DOWN_30  = +20.0
	DOWN_45  = +40.0
	DOWN_60	 = +60.0 # max +60 degrees

	# HEAD
	# STRAIGHT = 0
	# UP_40    = 1 
	# UP_20    = 2
	# DOWN_30  = 3
	# DOWN_45  = 4
	# DOWN_60  = 5 

	def __init__(self, desired_tilt):
		'''Constructor'''

		super(TiltHeadState, self).__init__(outcomes=['done', 'failed'])


		if not rospy.has_param("behavior/joint_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/joint_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/joint_controllers_name")

		# self._configs['flor']['same'] =  {
		# 	20: {'joint_name': 'neck_ry', 'joint_values': [+0.00], 'controller': 'neck_traj_controller'}, # max -40 degrees
		# 	21: {'joint_name': 'neck_ry', 'joint_values': [-40.0], 'controller': 'neck_traj_controller'},
		# 	22: {'joint_name': 'neck_ry', 'joint_values': [-20.0], 'controller': 'neck_traj_controller'},
		# 	23: {'joint_name': 'neck_ry', 'joint_values': [+20.0], 'controller': 'neck_traj_controller'},
		# 	24: {'joint_name': 'neck_ry', 'joint_values': [+40.0], 'controller': 'neck_traj_controller'},
		# 	25: {'joint_name': 'neck_ry', 'joint_values': [+60.0], 'controller': 'neck_traj_controller'}  # max +60 degrees
		# }

		self._action_topic = "/" + controller_namespace + \
							 "/neck_traj_controller" + "/follow_joint_trajectory"

		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		# Convert desired position to radians
		self._desired_tilt = math.radians(desired_tilt)

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
					Logger.logwarn('Joint trajectory request failed to execute: (%d) %s' % (result.error_code, result.error_string))
					self._failed = True
					return 'failed'
			else:
				Logger.logwarn('Wait for result returned True even though the result is %s' % str(result))
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering, create a neck trajectory and send the action goal.'''

		self._failed = False

		# Create neck point
		neck_point = JointTrajectoryPoint()
		neck_point.time_from_start = rospy.Duration.from_sec(1.0)
		neck_point.positions.append(self._desired_tilt)

		# Create neck trajectory
		neck_trajectory = JointTrajectory()
		neck_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
		neck_trajectory.joint_names = ['neck_ry']
		neck_trajectory.points.append(neck_point)

		# Create action goal
		action_goal = FollowJointTrajectoryGoal(trajectory = neck_trajectory)

		# execute the motion
		try: 
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send neck trajectory goal:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")
