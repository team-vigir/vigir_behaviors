#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_simple_footstep_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.start_record_logs_state import StartRecordLogsState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
from flexbe_states.wait_state import WaitState
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
from flexbe_atlas_states.create_step_goal_state import CreateStepGoalState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.plan_footsteps_state import PlanFootstepsState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_behavior_manipulation_config.manipulation_config_sm import ManipulationConfigSM
from vigir_behavior_locomotion_config.locomotion_config_sm import LocomotionConfigSM
from flexbe_states.input_state import InputState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import tf
import math

import rospy
import os
import time
# [/MANUAL_IMPORT]


'''
Created on Wed Nov 05 2014
@author: Philipp and Spyros
'''
class SimpleFootstepTestSM(Behavior):
	'''
	Behavior for testing footstep planning and execution. There are currently three testing modes: waypoint following, wide-stance/re-align, and testing plans modified on the OCS side.
	'''


	def __init__(self):
		super(SimpleFootstepTestSM, self).__init__()
		self.name = 'Simple Footstep Test'

		# parameters of this behavior
		self.add_parameter('topics_to_record', '')

		# references to used behaviors
		self.add_behavior(ManipulationConfigSM, 'Manipulation Config')
		self.add_behavior(LocomotionConfigSM, 'Locomotion Config')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		waypoints = [(1.0 , 0.0)] # list of (x,y)
		orientations = [0] # list of angles (degrees)
		# x:1329 y:218, x:1324 y:61
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pose = None
		_state_machine.userdata.target_pose_index = 0
		_state_machine.userdata.bagfile_name = '' # calculated

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# If the user doesn't specify any waypoints, default to these
		if not waypoints:
			waypoints = [(1.0, 0.0), (2.0, 0.26795), (2.73205, 1.0), (2.0, 0.26795)]
			orientations = [0, 30, 60, 30]

		orientations = [ang*math.pi/180 for ang in orientations] # convert to radians

		# Prepare a list of pose stamped waypoints for use by the footstep planning state
		self._poses_to_visit = list()
		
		for i in range(0, len(waypoints)):
			
			pt = Point(x=waypoints[i][0], y=waypoints[i][1])
			qt = tf.transformations.quaternion_from_euler(0, 0, orientations[i])
			p = Pose(position = pt, orientation = Quaternion(*qt))

			pose_stamped = PoseStamped(header = Header(frame_id = '/world'), pose = p)

			self._poses_to_visit.append(pose_stamped)

		# Topics of interest will be rosbag recorded and the bagfiles will be stored in the folder below
		logs_folder = os.path.expanduser('~/footstep_tests/')
		if not os.path.exists(logs_folder):
			os.makedirs(logs_folder)
		_state_machine.userdata.bagfile_name = logs_folder + "run_" + time.strftime("%Y-%m-%d-%H_%M") + ".bag"
		
		# [/MANUAL_CREATE]

		# x:730 y:38, x:328 y:117
		_sm_perform_walking_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose'])

		with _sm_perform_walking_0:
			# x:157 y:27
			OperatableStateMachine.add('Create_Step_Goal',
										CreateStepGoalState(pose_is_pelvis=False),
										transitions={'done': 'Plan_To_Next_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose': 'target_pose', 'step_goal': 'step_goal'})

			# x:401 y:188
			OperatableStateMachine.add('Print_The_Footstep_Plan',
										CalculationState(calculation=self.print_plan),
										transitions={'done': 'Execute_Footstep_Plan'},
										autonomy={'done': Autonomy.High},
										remapping={'input_value': 'plan_header', 'output_value': 'output_value'})

			# x:398 y:30
			OperatableStateMachine.add('Execute_Footstep_Plan',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:156 y:188
			OperatableStateMachine.add('Plan_To_Next_Pose',
										PlanFootstepsState(mode=PlanFootstepsState.MODE_STEP_NO_COLLISION),
										transitions={'planned': 'Print_The_Footstep_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})


		# x:1253 y:294, x:470 y:218
		_sm_test_ocs_modified_plan_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose_index'])

		with _sm_test_ocs_modified_plan_1:
			# x:116 y:56
			OperatableStateMachine.add('Set_Target_First_Pose',
										CalculationState(calculation=lambda x: self._poses_to_visit[0]),
										transitions={'done': 'Create_Goal'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'target_pose_index', 'output_value': 'target_pose'})

			# x:707 y:54
			OperatableStateMachine.add('Request_Plan',
										PlanFootstepsState(mode=PlanFootstepsState.MODE_STEP_NO_COLLISION),
										transitions={'planned': 'Print_Original_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})

			# x:1018 y:128
			OperatableStateMachine.add('Modify',
										InputState(request=InputState.FOOTSTEP_PLAN_HEADER, message='Modify plan (if necessary) and then confirm.'),
										transitions={'received': 'Print_Modified_Plan', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.High, 'aborted': Autonomy.Low, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'plan_header_new'})

			# x:971 y:358
			OperatableStateMachine.add('Execute',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header_new'})

			# x:431 y:56
			OperatableStateMachine.add('Create_Goal',
										CreateStepGoalState(pose_is_pelvis=False),
										transitions={'done': 'Request_Plan', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose': 'target_pose', 'step_goal': 'step_goal'})

			# x:1005 y:228
			OperatableStateMachine.add('Print_Modified_Plan',
										CalculationState(calculation=self.print_plan),
										transitions={'done': 'Execute'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_header_new', 'output_value': 'output_value'})

			# x:989 y:28
			OperatableStateMachine.add('Print_Original_Plan',
										CalculationState(calculation=self.print_plan),
										transitions={'done': 'Modify'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_header', 'output_value': 'output_value'})


		# x:8 y:123, x:561 y:146
		_sm_test_waypoint_following_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose_index'])

		with _sm_test_waypoint_following_2:
			# x:361 y:28
			OperatableStateMachine.add('Decide_Next_Target_Pose',
										DecisionState(outcomes=['continue', 'finished'], conditions=lambda x: 'continue' if x < len(waypoints) else 'finished'),
										transitions={'continue': 'Set_Next_Target_Pose', 'finished': 'finished'},
										autonomy={'continue': Autonomy.Low, 'finished': Autonomy.High},
										remapping={'input_value': 'target_pose_index'})

			# x:349 y:254
			OperatableStateMachine.add('Increment_To_Next_Target_Pose',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Decide_Next_Target_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'target_pose_index', 'output_value': 'target_pose_index'})

			# x:699 y:30
			OperatableStateMachine.add('Set_Next_Target_Pose',
										CalculationState(calculation=lambda x: self._poses_to_visit[x]),
										transitions={'done': 'Perform_Walking'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'target_pose_index', 'output_value': 'target_pose'})

			# x:679 y:254
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=True),
										transitions={'correct': 'Increment_To_Next_Target_Pose', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.High, 'incorrect': Autonomy.Low},
										remapping={'control_mode': 'control_mode'})

			# x:699 y:132
			OperatableStateMachine.add('Perform_Walking',
										_sm_perform_walking_0,
										transitions={'finished': 'Wait_For_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_pose': 'target_pose'})



		with _state_machine:
			# x:82 y:28
			OperatableStateMachine.add('Start_Recording',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Wait_For_Rosbag_Process'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'bagfile_name', 'rosbag_process': 'rosbag_process'})

			# x:1050 y:211
			OperatableStateMachine.add('Stop_Recording_If_Finished',
										StopRecordLogsState(),
										transitions={'stopped': 'finished'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:1035 y:52
			OperatableStateMachine.add('Stop_Recording_Before_Failed',
										StopRecordLogsState(),
										transitions={'stopped': 'failed'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:74 y:110
			OperatableStateMachine.add('Wait_For_Rosbag_Process',
										WaitState(wait_time=2),
										transitions={'done': 'Decide_Test_Type'},
										autonomy={'done': Autonomy.Off})

			# x:570 y:28
			OperatableStateMachine.add('Test_Waypoint_Following',
										_sm_test_waypoint_following_2,
										transitions={'finished': 'Stop_Recording_If_Finished', 'failed': 'Stop_Recording_Before_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_pose_index': 'target_pose_index'})

			# x:357 y:110
			OperatableStateMachine.add('Decide_Test_Type',
										OperatorDecisionState(outcomes=['waypoints', 'wide_stance', 'modified'], hint=None, suggestion=None),
										transitions={'waypoints': 'Test_Waypoint_Following', 'wide_stance': 'Manipulation Config', 'modified': 'Test_OCS_Modified_Plan'},
										autonomy={'waypoints': Autonomy.High, 'wide_stance': Autonomy.High, 'modified': Autonomy.High})

			# x:454 y:250
			OperatableStateMachine.add('Manipulation Config',
										self.use_behavior(ManipulationConfigSM, 'Manipulation Config'),
										transitions={'finished': 'Locomotion Config', 'failed': 'Stop_Recording_Before_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:704 y:252
			OperatableStateMachine.add('Locomotion Config',
										self.use_behavior(LocomotionConfigSM, 'Locomotion Config'),
										transitions={'finished': 'Stop_Recording_If_Finished', 'failed': 'Stop_Recording_Before_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:572 y:148
			OperatableStateMachine.add('Test_OCS_Modified_Plan',
										_sm_test_ocs_modified_plan_1,
										transitions={'finished': 'Stop_Recording_If_Finished', 'failed': 'Stop_Recording_Before_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_pose_index': 'target_pose_index'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def print_plan(self, plan):
		
		print 'Footstep Plan:\n', plan

	# [/MANUAL_FUNC]
