#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_grasp_object')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_states.calculation_state import CalculationState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.get_template_finger_config_state import GetTemplateFingerConfigState
from vigir_flexbe_states.hand_trajectory_state import HandTrajectoryState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.input_state import InputState
from vigir_flexbe_states.get_template_pregrasp_state import GetTemplatePregraspState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3Stamped, Vector3
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Fri Feb 27 2015
@author: Philipp Schillinger
'''
class GraspObjectSM(Behavior):
	'''
	Behavior to perform grasping. Robot has to be in range of the object.
	'''


	def __init__(self):
		super(GraspObjectSM, self).__init__()
		self.name = 'Grasp Object'

		# parameters of this behavior
		self.add_parameter('hand_type', 'robotiq')
		self.add_parameter('hand_side', 'left')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 866 352 /Go_to_Grasp
		# Need to adjust pregrasp pose for new grasp.

		# O 452 33 
		# Close fingers to avoid collisions and reduce risk of damage when colliding

		# O 496 391 /Perform_Grasp
		# Go back to pregrasp pose first

		# O 699 12 /Go_to_Grasp
		# Allow operator to precisely align the template.



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		# x:1033 y:440, x:359 y:535
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.template_id = None # provide or request

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:819 y:235, x:305 y:73
		_sm_go_to_pregrasp_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose'])

		with _sm_go_to_pregrasp_0:
			# x:27 y:68
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'Inform_Pregrasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:269 y:153
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:537 y:228
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:25 y:328
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:266 y:228
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:266 y:327
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Pregrasp_Info', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:110 y:343, x:533 y:169, x:728 y:40
		_sm_perform_grasp_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'next'], input_keys=['hand_side', 'grasp_preference', 'template_id', 'pregrasp_pose'], output_keys=['grasp_preference'])

		with _sm_perform_grasp_1:
			# x:68 y:76
			OperatableStateMachine.add('Get_Finger_Configuration',
										GetTemplateFingerConfigState(),
										transitions={'done': 'Close_Fingers', 'failed': 'failed', 'not_available': 'Inform_Closing_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'finger_config': 'finger_config'})

			# x:293 y:328
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_Back_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_waypoints'})

			# x:496 y:328
			OperatableStateMachine.add('Plan_Back_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_Back_To_Pregrasp_Pose', 'incomplete': 'Move_Back_To_Pregrasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'waypoints': 'pregrasp_waypoints', 'hand': 'hand_side', 'frame_id': 'pregrasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:662 y:228
			OperatableStateMachine.add('Move_Back_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Increase_Preference_Index', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:296 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_frame_id'})

			# x:673 y:128
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'next'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:81 y:228
			OperatableStateMachine.add('Close_Fingers',
										HandTrajectoryState(hand_type=self.hand_type),
										transitions={'done': 'finished', 'failed': 'Extract_Frame_Id'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'finger_trajectory': 'finger_config', 'hand_side': 'hand_side'})

			# x:490 y:75
			OperatableStateMachine.add('Inform_Closing_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:1041 y:56, x:257 y:85, x:1035 y:196
		_sm_go_to_grasp_2 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference'])

		with _sm_go_to_grasp_2:
			# x:33 y:49
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low, 'not_available': Autonomy.Low},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'grasp_pose'})

			# x:40 y:293
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})

			# x:242 y:292
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp_Pose', 'incomplete': 'Move_To_Grasp_Pose', 'failed': 'Decide_Which_Grasp'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:494 y:175
			OperatableStateMachine.add('Move_To_Grasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Decide_Which_Grasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:226 y:177
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:970 y:294
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'again'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})

			# x:727 y:50
			OperatableStateMachine.add('Optional_Template_Adjustment',
										OperatorDecisionState(outcomes=["grasp", "pregrasp", "skip"], hint="Consider adjusting the template's pose", suggestion="skip"),
										transitions={'grasp': 'Get_Grasp_Info', 'pregrasp': 'again', 'skip': 'finished'},
										autonomy={'grasp': Autonomy.Full, 'pregrasp': Autonomy.Full, 'skip': Autonomy.High})

			# x:754 y:294
			OperatableStateMachine.add('Decide_Which_Grasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same grasp or the next one?', suggestion='same'),
										transitions={'same': 'Optional_Template_Adjustment', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})



		with _state_machine:
			# x:24 y:78
			OperatableStateMachine.add('Decide_Request_Template',
										DecisionState(outcomes=['request', 'continue'], conditions=lambda x: 'continue' if x is not None else 'request'),
										transitions={'request': 'Request_Template', 'continue': 'Go_to_Pregrasp'},
										autonomy={'request': Autonomy.Low, 'continue': Autonomy.Off},
										remapping={'input_value': 'template_id'})

			# x:849 y:224
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_2,
										transitions={'finished': 'Perform_Grasp', 'failed': 'Grasp_Manually', 'again': 'Close_Fingers'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id'})

			# x:845 y:81
			OperatableStateMachine.add('Perform_Grasp',
										_sm_perform_grasp_1,
										transitions={'finished': 'finished', 'failed': 'Grasp_Manually', 'next': 'Close_Fingers'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'next': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:506 y:228
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0.0),
										transitions={'done': 'Go_to_Grasp', 'failed': 'Grasp_Manually'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:506 y:84
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1.0),
										transitions={'done': 'Go_to_Pregrasp', 'failed': 'Grasp_Manually'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:324 y:428
			OperatableStateMachine.add('Grasp_Manually',
										OperatorDecisionState(outcomes=["fingers_closed", "abort"], hint="Grasp the object manually, continue when fingers are closed.", suggestion=None),
										transitions={'fingers_closed': 'finished', 'abort': 'failed'},
										autonomy={'fingers_closed': Autonomy.Full, 'abort': Autonomy.Full})

			# x:34 y:178
			OperatableStateMachine.add('Request_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Specify target template"),
										transitions={'received': 'Go_to_Pregrasp', 'aborted': 'Grasp_Manually', 'no_connection': 'Grasp_Manually', 'data_error': 'Grasp_Manually'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:236 y:122
			OperatableStateMachine.add('Go_to_Pregrasp',
										_sm_go_to_pregrasp_0,
										transitions={'finished': 'Open_Fingers', 'failed': 'Grasp_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'pregrasp_pose': 'pregrasp_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
