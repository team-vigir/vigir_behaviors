#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_open_door_thor_side')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.calculation_state import CalculationState
from flexbe_states.log_state import LogState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from vigir_flexbe_states.get_template_grasp_state import GetTemplateGraspState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.look_at_target_state import LookAtTargetState
from vigir_flexbe_states.get_template_pregrasp_state import GetTemplatePregraspState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.plan_footsteps_state import PlanFootstepsState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from vigir_flexbe_states.create_step_goal_state import CreateStepGoalState
from vigir_flexbe_states.get_pose_in_frame_state import GetPoseInFrameState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Sun May 24 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class OpenDoorTHORSideSM(Behavior):
	'''
	Performs the open door task challenge.
	'''


	def __init__(self):
		super(OpenDoorTHORSideSM, self).__init__()
		self.name = 'Open Door THOR Side'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('parameter_set', 'drc_step_2D')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk_to_Template')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 980 304 /Prepare_to_Open
		# Is it worth having a sub behavior for this preference game?

		# O 671 126 /Open_Door/Unlock_Door
		# make sure the handle is turned far enough



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		handle_down_affordance = 'turnCW' if self.hand_side == 'left' else 'turnCCW'
		door_affordance = 'push'
		handle_up_affordance = 'turnCCW' if self.hand_side == 'left' else 'turnCW'
		turn_threshold = 0.6
		# x:433 y:590, x:333 y:340
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.waypoint_distance = 1.5 # meters

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:749 y:46, x:534 y:88
		_sm_next_pre_grasp_pose_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_next_pre_grasp_pose_0:
			# x:27 y:78
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:296 y:278
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Prerasp_Pose', 'incomplete': 'Move_To_Prerasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:278
			OperatableStateMachine.add('Move_To_Prerasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:237 y:128
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_frame_id'})

			# x:41 y:278
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_waypoints'})


		# x:983 y:290, x:433 y:90
		_sm_back_to_pre_grasp_pose_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_back_to_pre_grasp_pose_1:
			# x:27 y:78
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:296 y:278
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:278
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:237 y:128
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_frame_id'})

			# x:41 y:278
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_waypoints'})


		# x:683 y:90, x:283 y:90, x:1083 y:90
		_sm_go_to_grasp_pose_2 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_go_to_grasp_pose_2:
			# x:33 y:49
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'grasp_pose'})

			# x:41 y:278
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})

			# x:296 y:278
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp_Pose', 'incomplete': 'Move_To_Grasp_Pose', 'failed': 'Decide_Which_Grasp'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:278
			OperatableStateMachine.add('Move_To_Grasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Decide_Which_Grasp'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:226 y:177
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})

			# x:612 y:178
			OperatableStateMachine.add('Optional_Template_Adjustment',
										OperatorDecisionState(outcomes=["grasp", "pregrasp", "skip"], hint="Consider adjusting the template's pose", suggestion="skip"),
										transitions={'grasp': 'Get_Grasp_Info', 'pregrasp': 'again', 'skip': 'finished'},
										autonomy={'grasp': Autonomy.Full, 'pregrasp': Autonomy.Full, 'skip': Autonomy.High})

			# x:836 y:428
			OperatableStateMachine.add('Decide_Which_Grasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same grasp or the next one?', suggestion='same'),
										transitions={'same': 'Optional_Template_Adjustment', 'next': 'again'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:133 y:540, x:533 y:90, x:733 y:240
		_sm_go_to_pre_grasp_pose_3 = OperatableStateMachine(outcomes=['finished', 'failed', 'adjust'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose', 'template_id'])

		with _sm_go_to_pre_grasp_pose_3:
			# x:77 y:78
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'Inform_Pregrasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:330 y:28
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:76 y:378
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:423 y:178
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'adjust'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:79 y:228
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:429 y:328
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'adjust', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:733 y:390, x:333 y:40
		_sm_hand_back_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_hand_back_4:
			# x:84 y:28
			OperatableStateMachine.add('Init_Grasp_Preference',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Get_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'grasp_preference'})

			# x:380 y:128
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:476 y:378
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Increase_Preference_Index'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:73 y:428
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:191 y:328
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_Back_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_waypoints'})

			# x:344 y:189
			OperatableStateMachine.add('Plan_Back_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'pregrasp_waypoints', 'hand': 'hand_side', 'frame_id': 'pregrasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:196 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_frame_id'})

			# x:50 y:124
			OperatableStateMachine.add('Get_Grasp_Pose',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'Inform_Pregrasp_Failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'pregrasp_pose'})


		# x:822 y:141, x:393 y:61
		_sm_push_door_open_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'])

		with _sm_push_door_open_5:
			# x:134 y:68
			OperatableStateMachine.add('Move_Pre_Push_Side',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_PRE_PUSH_SIDE, vel_scaling=0.1, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Move_Push_Side', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:126 y:178
			OperatableStateMachine.add('Move_Push_Side',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_PUSH_SIDE, vel_scaling=0.05, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Move_Push_Side_Walk', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:526 y:128
			OperatableStateMachine.add('Move_Push_Side_Walk',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_PUSH_SIDE_WALK, vel_scaling=0.1, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})


		# x:633 y:490, x:197 y:243
		_sm_place_handle_hand_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['door_template_id', 'hand_side', 'grasp_preference'])

		with _sm_place_handle_hand_6:
			# x:68 y:72
			OperatableStateMachine.add('Go_to_Pre_Grasp_Pose',
										_sm_go_to_pre_grasp_pose_3,
										transitions={'finished': 'Decide_If_Open', 'failed': 'failed', 'adjust': 'Next_Pre_Grasp_Pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'adjust': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:479 y:264
			OperatableStateMachine.add('Go_to_Grasp_Pose',
										_sm_go_to_grasp_pose_2,
										transitions={'finished': 'Decide_If_Grasp', 'failed': 'failed', 'again': 'Back_to_Pre_Grasp_Pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:498 y:78
			OperatableStateMachine.add('Decide_If_Open',
										DecisionState(outcomes=["open", "push"], conditions=lambda x: "open" if x == 0 else "push"),
										transitions={'open': 'Open_Fingers', 'push': 'Go_to_Grasp_Pose'},
										autonomy={'open': Autonomy.Full, 'push': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})

			# x:282 y:178
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'Go_to_Grasp_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'hand_side': 'hand_side'})

			# x:63 y:322
			OperatableStateMachine.add('Back_to_Pre_Grasp_Pose',
										_sm_back_to_pre_grasp_pose_1,
										transitions={'finished': 'Increase_Preference_Index', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:498 y:378
			OperatableStateMachine.add('Decide_If_Grasp',
										DecisionState(outcomes=["grasp", "push"], conditions=lambda x: "grasp" if x == 0 else "push"),
										transitions={'grasp': 'Close_Fingers', 'push': 'finished'},
										autonomy={'grasp': Autonomy.High, 'push': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})

			# x:332 y:478
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:271 y:22
			OperatableStateMachine.add('Next_Pre_Grasp_Pose',
										_sm_next_pre_grasp_pose_0,
										transitions={'finished': 'Decide_If_Open', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:23 y:191
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Go_to_Pre_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})


		# x:1041 y:400, x:987 y:18
		_sm_unlock_door_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'none', 'door_template_id'], output_keys=['turn_fraction'])

		with _sm_unlock_door_7:
			# x:64 y:28
			OperatableStateMachine.add('Get_Handle_Affordance_Down',
										GetTemplateAffordanceState(identifier=handle_down_affordance),
										transitions={'done': 'Plan_Turn_Handle_Down', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:992 y:178
			OperatableStateMachine.add('Plan_Push_Door',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Push_Door', 'incomplete': 'Push_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'door_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:973 y:78
			OperatableStateMachine.add('Get_Push_Affordance',
										GetTemplateAffordanceState(identifier=door_affordance),
										transitions={'done': 'Plan_Push_Door', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'door_affordance'})

			# x:277 y:78
			OperatableStateMachine.add('Plan_Turn_Handle_Down',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Store_Turn_Down', 'incomplete': 'Decide_Execute_Incomplete', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:676 y:78
			OperatableStateMachine.add('Turn_Handle',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Get_Push_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:976 y:278
			OperatableStateMachine.add('Push_Door',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:71 y:263
			OperatableStateMachine.add('Get_Handle_Affordance_Up',
										GetTemplateAffordanceState(identifier=handle_up_affordance),
										transitions={'done': 'Plan_Turn_Handle_Up', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:284 y:270
			OperatableStateMachine.add('Plan_Turn_Handle_Up',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Store_Turn_Up', 'incomplete': 'Store_Turn_Up', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:70 y:178
			OperatableStateMachine.add('Decide_Execute_Incomplete',
										DecisionState(outcomes=["up", "down"], conditions=lambda x: "up" if x > turn_threshold else "down"),
										transitions={'up': 'Store_Turn_Down', 'down': 'Get_Handle_Affordance_Up'},
										autonomy={'up': Autonomy.Low, 'down': Autonomy.Low},
										remapping={'input_value': 'plan_fraction'})

			# x:494 y:78
			OperatableStateMachine.add('Store_Turn_Down',
										CalculationState(calculation=lambda x: x),
										transitions={'done': 'Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_fraction', 'output_value': 'turn_fraction'})

			# x:501 y:178
			OperatableStateMachine.add('Store_Turn_Up',
										CalculationState(calculation=lambda x: -x),
										transitions={'done': 'Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_fraction', 'output_value': 'turn_fraction'})


		# x:783 y:490, x:133 y:390
		_sm_release_handle_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none', 'turn_fraction', 'grasp_preference'])

		with _sm_release_handle_8:
			# x:33 y:120
			OperatableStateMachine.add('Decide_Turn_Direction',
										DecisionState(outcomes=["up", "down"], conditions=lambda x: "up" if x > 0 else "down"),
										transitions={'up': 'Get_Handle_Affordance_Up', 'down': 'Get_Handle_Affordance_Down'},
										autonomy={'up': Autonomy.Low, 'down': Autonomy.Low},
										remapping={'input_value': 'turn_fraction'})

			# x:744 y:372
			OperatableStateMachine.add('Hand_Back',
										_sm_hand_back_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:742 y:128
			OperatableStateMachine.add('Plan_Turn_Handle',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Turn_Handle', 'incomplete': 'Turn_Handle', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:926 y:128
			OperatableStateMachine.add('Turn_Handle',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_If_Open', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:782 y:278
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0.0),
										transitions={'done': 'Decide_Retract_Hand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:455 y:128
			OperatableStateMachine.add('Reduce_Affordance_Displacement',
										FlexibleCalculationState(calculation=self.scale_affordance, input_keys=["affordance", "fraction"]),
										transitions={'done': 'Plan_Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'affordance': 'handle_affordance', 'fraction': 'turn_fraction', 'output_value': 'scaled_affordance'})

			# x:214 y:178
			OperatableStateMachine.add('Get_Handle_Affordance_Down',
										GetTemplateAffordanceState(identifier=handle_down_affordance),
										transitions={'done': 'Reduce_Affordance_Displacement', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:221 y:78
			OperatableStateMachine.add('Get_Handle_Affordance_Up',
										GetTemplateAffordanceState(identifier=handle_up_affordance),
										transitions={'done': 'Reduce_Affordance_Displacement', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:936 y:378
			OperatableStateMachine.add('Decide_Retract_Hand',
										OperatorDecisionState(outcomes=['keep', 'back'], hint="Take hand back from handle?", suggestion='keep'),
										transitions={'keep': 'finished', 'back': 'Hand_Back'},
										autonomy={'keep': Autonomy.High, 'back': Autonomy.Full})

			# x:950 y:213
			OperatableStateMachine.add('Decide_If_Open',
										DecisionState(outcomes=["open", "retract"], conditions=lambda x: "open" if x == 0 else "retract"),
										transitions={'open': 'Open_Fingers', 'retract': 'Decide_Retract_Hand'},
										autonomy={'open': Autonomy.High, 'retract': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})


		# x:124 y:577, x:483 y:290
		_sm_traverse_door_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint_distance', 'none', 'hand_side'])

		with _sm_traverse_door_9:
			# x:56 y:60
			OperatableStateMachine.add('Generate_Traversing_Waypoint',
										FlexibleCalculationState(calculation=self.generate_traversing_waypoint, input_keys=["hand_side" , "waypoint_distance"]),
										transitions={'done': 'Convert_Waypoint_Frame'},
										autonomy={'done': Autonomy.Off},
										remapping={'hand_side': 'hand_side', 'waypoint_distance': 'waypoint_distance', 'output_value': 'waypoint_pelvis'})

			# x:77 y:358
			OperatableStateMachine.add('Plan_Through_Door',
										PlanFootstepsState(mode=PlanFootstepsState.MODE_STEP_NO_COLLISION),
										transitions={'planned': 'Go_Through_Door', 'failed': 'Take_Arms_Side'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})

			# x:70 y:450
			OperatableStateMachine.add('Go_Through_Door',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'Take_Arms_Side'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})

			# x:77 y:256
			OperatableStateMachine.add('Create_Step_Goal',
										CreateStepGoalState(pose_is_pelvis=True),
										transitions={'done': 'Plan_Through_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'target_pose': 'waypoint_world', 'step_goal': 'step_goal'})

			# x:286 y:333
			OperatableStateMachine.add('Take_Arms_Side',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.BOTH_ARMS_SIDES, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Plan_Through_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:68 y:170
			OperatableStateMachine.add('Convert_Waypoint_Frame',
										GetPoseInFrameState(target_frame='world'),
										transitions={'done': 'Create_Step_Goal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'pose_in': 'waypoint_pelvis', 'pose_out': 'waypoint_world'})


		# x:683 y:890, x:133 y:690
		_sm_open_door_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'door_template_id', 'hand_side', 'grasp_preference'])

		with _sm_open_door_10:
			# x:87 y:28
			OperatableStateMachine.add('Set_Template_Frame',
										CalculationState(calculation=lambda x: 'template_tf_' + str(x)),
										transitions={'done': 'Look_At_Handle_Hand'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'door_template_id', 'output_value': 'template_frame'})

			# x:364 y:95
			OperatableStateMachine.add('Log_Try_To_Open',
										LogState(text='Will now try to open', severity=Logger.REPORT_INFO),
										transitions={'done': 'Unlock_Door'},
										autonomy={'done': Autonomy.High})

			# x:576 y:273
			OperatableStateMachine.add('Release_Handle',
										_sm_release_handle_8,
										transitions={'finished': 'Ask_If_Push', 'failed': 'Log_Remove_Hand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'none': 'none', 'turn_fraction': 'turn_fraction', 'grasp_preference': 'grasp_preference'})

			# x:777 y:277
			OperatableStateMachine.add('Log_Remove_Hand',
										LogState(text='Remove hand from handle', severity=Logger.REPORT_HINT),
										transitions={'done': 'Ask_If_Push'},
										autonomy={'done': Autonomy.Full})

			# x:394 y:222
			OperatableStateMachine.add('Unlock_Door',
										_sm_unlock_door_7,
										transitions={'finished': 'Ask_for_Retry', 'failed': 'Ask_for_Retry'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'none': 'none', 'door_template_id': 'door_template_id', 'turn_fraction': 'turn_fraction'})

			# x:597 y:109
			OperatableStateMachine.add('Ask_for_Retry',
										OperatorDecisionState(outcomes=["release_handle", "retry"], hint="Now release handle?", suggestion="release_handle"),
										transitions={'release_handle': 'Release_Handle', 'retry': 'Log_Try_To_Open'},
										autonomy={'release_handle': Autonomy.High, 'retry': Autonomy.Full})

			# x:687 y:378
			OperatableStateMachine.add('Ask_If_Push',
										OperatorDecisionState(outcomes=['push', 'grasp_again'], hint='Is the door slightly open?', suggestion='push'),
										transitions={'push': 'Look_Trough_Door', 'grasp_again': 'Place_Handle_Hand'},
										autonomy={'push': Autonomy.High, 'grasp_again': Autonomy.Full})

			# x:83 y:128
			OperatableStateMachine.add('Look_At_Handle_Hand',
										LookAtTargetState(),
										transitions={'done': 'Align_Door_Log'},
										autonomy={'done': Autonomy.Off},
										remapping={'frame': 'template_frame'})

			# x:77 y:372
			OperatableStateMachine.add('Place_Handle_Hand',
										_sm_place_handle_hand_6,
										transitions={'finished': 'Log_Try_To_Open', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'door_template_id': 'door_template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference'})

			# x:99 y:220
			OperatableStateMachine.add('Align_Door_Log',
										LogState(text="Adjust pose of the door template", severity=Logger.REPORT_HINT),
										transitions={'done': 'Place_Handle_Hand'},
										autonomy={'done': Autonomy.Full})

			# x:745 y:497
			OperatableStateMachine.add('Look_Trough_Door',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.LOOK_SIDE, vel_scaling=0.5, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Push_Door_Open', 'failed': 'Push_Door_Open'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:638 y:609
			OperatableStateMachine.add('Push_Door_Open',
										_sm_push_door_open_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side'})



		with _state_machine:
			# x:82 y:28
			OperatableStateMachine.add('Get_Door_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the DOOR template."),
										transitions={'received': 'Decide_Walking', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'door_template_id'})

			# x:616 y:478
			OperatableStateMachine.add('Go_To_Stand_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND_MANIPULATE),
										transitions={'changed': 'Traverse_Door', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:81 y:272
			OperatableStateMachine.add('Walk_to_Template',
										self.use_behavior(WalktoTemplateSM, 'Walk_to_Template'),
										transitions={'finished': 'Manipulate_On', 'failed': 'failed', 'aborted': 'Manipulate_On'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'door_template_id'})

			# x:316 y:128
			OperatableStateMachine.add('Manipulate_On',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Open_Door', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:644 y:122
			OperatableStateMachine.add('Open_Door',
										_sm_open_door_10,
										transitions={'finished': 'Wait_For_Gather_Data', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'door_template_id': 'door_template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference'})

			# x:634 y:278
			OperatableStateMachine.add('Wait_For_Gather_Data',
										LogState(text='Gather data from inside', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_To_Stand_Manipulate'},
										autonomy={'done': Autonomy.Full})

			# x:390 y:472
			OperatableStateMachine.add('Traverse_Door',
										_sm_traverse_door_9,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint_distance': 'waypoint_distance', 'none': 'none', 'hand_side': 'hand_side'})

			# x:87 y:128
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Walk to the door?", suggestion="walk"),
										transitions={'walk': 'Walk_to_Template', 'stand': 'Manipulate_On'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def calc_open_finger_traj(self, pre_grasp):
		'''Creates a JointTrajectory msg that corresponds to the open fingers configuration (all zeros).

		Uses the grasp pose message to infer the correct joint names.
		'''

		# Set the finger joint configuration that corresponds to an open hand
		open_config = JointTrajectoryPoint()
		open_config.positions = [0.0] * 11
		open_config.time_from_start = rospy.Duration.from_sec(0.1)

		open_fingers_traj = JointTrajectory()
		open_fingers_traj.points.append(open_config)
		open_fingers_traj.joint_names = pre_grasp.pre_grasp_posture.joint_names

		print(open_fingers_traj)

		return open_fingers_traj


	def generate_traversing_waypoint(self, input_keys):
		hand_side = input_keys[0]
		distance = inpu_keys[1]

		pose = PoseStamped()
		pose.header.frame_id = "pelvis"
		pose.pose.orientation.w = 1
		if hand_side == "left":
			pose.pose.position.y = distance
		else:
			pose.pose.position.y = -distance

		return pose 


	def scale_affordance(self, input_list):
		affordance = input_list[0]
		fraction = input_list[1]
		affordance.displacement *= abs(fraction)
		return affordance
	
	# [/MANUAL_FUNC]
