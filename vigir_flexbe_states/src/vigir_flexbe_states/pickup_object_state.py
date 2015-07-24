#!/usr/bin/env python

import math
import rospy
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

'''
Created on 01/23/2015

@author: Spyros Maniatopoulos
'''
class PickupObjectState(EventState):
	'''
	Executes a given hand trajectory, i.e., a request to open or close the fingers.

	Keyword arguments:
    object_to_grasp -- Name as known in the planning scene
    planning_group -- Which move group should be used to plan for pickup
    '''

	def __init__(self, object_to_grasp, planning_group):
		'''Constructor'''
		super(PickupObjectState, self).__init__(outcomes=['done', 'failed'],
												input_keys=['possible_grasps', 'support_surface_name'])
		self._object = object_to_grasp
		self._planning_group = planning_group
		self._action_topic = "/pickup" #FIX: Get correct action topic

		self._client = actionlib.SimpleActionClient(self._action_topic, PickupAction)
		self._client.wait_for_server(rospy.Duration.from_sec(10))

		self._failed = False
	
	def execute(self, userdata):
		'''During execution of the state, keep checking for action servers response'''

		if self._failed:
			return 'failed'

		if self._client.wait_for_result(rospy.Duration.from_sec(0.1)):
			result = self._client.get_result()
			Logger.loginfo('Action server says: %s' % result.error_code.val)
			
			if result.error_code.val == MoveItErrorCodes.SUCCESS:
				return 'success'
			else:
				Logger.logwarn('Pickup action request failed to execute: (%d) %s' % (result.error_code.val, result.error_string)) #FIX: Is there an error_string?
				return 'failed'


	def on_enter(self, userdata):
		'''Upon entering the state, create and send the action goal message'''

		# Create goal message
		action = PickupGoal()
		# action.request = MotionPlanRequest()
		# action.request.goal_constraints = [Constraints()]
		# action.request.goal_constraints[0].joint_constraints = []

		# for i in range(min(len(self._joint_names), len(userdata.target_joint_config))):
		# 	action.request.goal_constraints[0].joint_constraints.append(JointConstraint(joint_name = self._joint_names[i], position = userdata.target_joint_config[i]))

		# action.request.group_name = self._planning_group

		# Send goal to action server for execution
		try: 
			Logger.loginfo("Picking up %s using %s" % (self._object, self._planning_group))
			self._client.send_goal(action)
		except Exception as e:
			Logger.logwarn('Was unable to execute pickup action request:\n%s' % str(e))
			self._failed = True


