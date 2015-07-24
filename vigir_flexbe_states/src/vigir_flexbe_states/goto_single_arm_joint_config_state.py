#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

"""
Created on 05/18/2015

@author: Spyros Maniatopoulos
"""

class GotoSingleArmJointConfigState(EventState):
	"""
	Directly commands the trajectory/joint controllers to move a 
	single joint to the desired configuration.

	-- target_config 	int 		Identifier of the pre-defined pose to be used.
	-- arm_side			string		Arm side {left, right}
 
	># current_config 	dict		The current arm joint positions
									joint_names string[] : joint_values[]

	<= done 						Successfully executed the motion.
	<= failed 						Failed to execute the motion.

	"""

	# Wrists
	WRIST_CCW = 11
	WRIST_CW  = 12

	# Forearms
	# ...

	
	def __init__(self, arm_side, target_config, time = 2.0):
		"""Constructor"""
		super(GotoSingleArmJointConfigState, self).__init__(outcomes = ['done', 'failed'],
															input_keys = ['current_config'])

		if not rospy.has_param("behavior/robot_namespace"):
			Logger.logerr("Need to specify parameter behavior/robot_namespace at the parameter server")
			return
		
		self._robot = rospy.get_param("behavior/robot_namespace")

		if not rospy.has_param("behavior/joint_controllers_name"):
			Logger.logerr("Need to specify parameter behavior/joint_controllers_name at the parameter server")
			return

		controller_namespace = rospy.get_param("behavior/joint_controllers_name")

		################################ ATLAS ################################
		self._configs = dict()
		self._configs['flor'] = dict()
		self._configs['flor']['left'] =  {
			11: {'joint_name': 'l_arm_wry2', 'joint_value': -2.5},
			12: {'joint_name': 'l_arm_wry2', 'joint_value': +2.5}
		}
		self._configs['flor']['right'] =  {					 
			11: {'joint_name': 'r_arm_wry2', 'joint_value': +2.5},
			12: {'joint_name': 'r_arm_wry2', 'joint_value': -2.5}
		}
		################################ THOR #################################
		self._configs['thor_mang'] = dict()
		self._configs['thor_mang']['left'] = {
			11: {'joint_name': 'l_wrist_yaw2', 'joint_value': 3.84},
			12: {'joint_name': 'l_wrist_yaw2', 'joint_value': -3.84}
		}
		self._configs['thor_mang']['right'] = {
			11: {'joint_name': 'r_wrist_yaw2', 'joint_value': -3.84},
			12: {'joint_name': 'r_wrist_yaw2', 'joint_value': 3.84}
		}		
		#######################################################################

		self._joint_name = self._configs[self._robot][arm_side][target_config]['joint_name']
		self._joint_value = self._configs[self._robot][arm_side][target_config]['joint_value']
		self._time = time

		self._action_topic = "/" + controller_namespace + "/" + arm_side + \
							 "_arm_traj_controller" + "/follow_joint_trajectory"
		
		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})
		
		self._failed = False


	def execute(self, userdata):
		"""Execute this state"""
		
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result:
				if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
					return 'done'
				else:
					Logger.logwarn('Joint trajectory failed to execute (%d). Reason: %s' % (result.error_code, result.error_string))
					self._failed = True
					return 'failed'
			else:
				Logger.logwarn('Wait for result returned True even though the result is %s' % str(result))
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		'''On enter, create and send the follow joint trajectory action goal.'''

		self._failed = False

		current_config = userdata.current_config

		# Get the index of the joint whose value will be replaced
		index_of_joint = current_config['joint_names'].index(self._joint_name)

		# Replace the old joint value with the target_config's one
		new_values = current_config['joint_values']
		new_values[index_of_joint] = self._joint_value

		# Create trajectory point out of the revised joint values
		point = JointTrajectoryPoint(
						positions = new_values,
						time_from_start = rospy.Duration.from_sec(self._time))

		# Create trajectory message
		trajectory = JointTrajectory(
						joint_names = current_config['joint_names'],
						points = [point])

		action_goal = FollowJointTrajectoryGoal(trajectory = trajectory)

		# execute the motion
		try: 
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send trajectory action goal:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		'''Destructor'''
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")
