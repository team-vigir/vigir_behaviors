#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_surprise_push_button')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_grasp_object.grasp_object_sm import GraspObjectSM
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_atlas_states.finger_configuration_state import FingerConfigurationState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_atlas_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_states.calculation_state import CalculationState
from flexbe_atlas_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from flexbe_atlas_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_states.log_state import LogState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 01 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class SurprisePushButtonSM(Behavior):
	'''
	to push the button in the surprise task
	'''


	def __init__(self):
		super(SurprisePushButtonSM, self).__init__()
		self.name = 'Surprise Push Button'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('hand_side_push', 'right')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk_To_Template')
		self.add_behavior(GraspObjectSM, 'Open_Box/Grasp_Box')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		box_part_1_affordance = "open_part_1"
		box_part_2_affordance = "open_part_2"
		# x:211 y:659, x:307 y:517
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.step_back_distance = 1.0 # meters
		_state_machine.userdata.hand_side_push = self.hand_side_push

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:758 y:166, x:513 y:45
		_sm_back_to_pregrasp_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'grasp_preference'])

		with _sm_back_to_pregrasp_0:
			# x:33 y:49
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'grasp_pose'})

			# x:40 y:293
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})

			# x:242 y:292
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'Get_Pregrasp_Info'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:526 y:228
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})


		# x:634 y:104, x:343 y:102, x:1013 y:188
		_sm_go_to_grasp_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference'])

		with _sm_go_to_grasp_1:
			# x:33 y:49
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
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
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:494 y:175
			OperatableStateMachine.add('Move_To_Grasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Grasp'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

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
										autonomy={'same': Autonomy.High, 'next': Autonomy.Full})


		# x:772 y:165, x:514 y:83
		_sm_go_to_pregrasp_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose'])

		with _sm_go_to_pregrasp_2:
			# x:83 y:68
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:531 y:294
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:25 y:328
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:314 y:219
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:309 y:395
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Pregrasp_Info', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:30 y:365, x:130 y:365
		_sm_perform_step_back_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['step_back_distance'])

		with _sm_perform_step_back_3:
			# x:78 y:78
			OperatableStateMachine.add('Plan_Steps_Back',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Do_Steps_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'distance': 'step_back_distance', 'plan_header': 'plan_header'})

			# x:74 y:228
			OperatableStateMachine.add('Do_Steps_Back',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})


		# x:722 y:376, x:130 y:365
		_sm_push_button_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'grasp_preference'])

		with _sm_push_button_4:
			# x:94 y:57
			OperatableStateMachine.add('Go_to_Pregrasp',
										_sm_go_to_pregrasp_2,
										transitions={'finished': 'Go_to_Grasp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:362 y:206
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_1,
										transitions={'finished': 'Back_to_Pregrasp', 'failed': 'failed', 'again': 'Go_to_Pregrasp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id'})

			# x:619 y:282
			OperatableStateMachine.add('Back_to_Pregrasp',
										_sm_back_to_pregrasp_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference'})


		# x:1187 y:28, x:856 y:73
		_sm_open_box_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_open_box_5:
			# x:93 y:98
			OperatableStateMachine.add('Grasp_Box',
										self.use_behavior(GraspObjectSM, 'Open_Box/Grasp_Box'),
										transitions={'finished': 'Get_Open_Part_1_Affordance', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:275 y:250
			OperatableStateMachine.add('Plan_Open_Part_1_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Open_Part_1_Affordance', 'incomplete': 'Execute_Open_Part_1_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'box_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:280 y:331
			OperatableStateMachine.add('Execute_Open_Part_1_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Get_Open_Part_2_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:274 y:178
			OperatableStateMachine.add('Get_Open_Part_1_Affordance',
										GetTemplateAffordanceState(identifier=box_part_1_affordance),
										transitions={'done': 'Plan_Open_Part_1_Affordance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'box_affordance'})

			# x:500 y:462
			OperatableStateMachine.add('Plan_Open_Part_2_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Open_Part_2_Affordance', 'incomplete': 'Execute_Open_Part_2_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'box_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:728 y:461
			OperatableStateMachine.add('Execute_Open_Part_2_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Open_Fingers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:365 y:396
			OperatableStateMachine.add('Get_Open_Part_2_Affordance',
										GetTemplateAffordanceState(identifier=box_part_2_affordance),
										transitions={'done': 'Plan_Open_Part_2_Affordance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'box_affordance'})

			# x:964 y:433
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'hand_side': 'hand_side'})



		with _state_machine:
			# x:73 y:78
			OperatableStateMachine.add('Request_Box_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place Box template"),
										transitions={'received': 'Decide_Walking', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'template_id'})

			# x:330 y:172
			OperatableStateMachine.add('Walk_To_Template',
										self.use_behavior(WalktoTemplateSM, 'Walk_To_Template'),
										transitions={'finished': 'Set_Manipulate', 'failed': 'failed', 'aborted': 'Set_Manipulate'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:337 y:78
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Walk to template?", suggestion="walk"),
										transitions={'walk': 'Walk_To_Template', 'stand': 'Set_Manipulate'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:566 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Open_Box', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:794 y:72
			OperatableStateMachine.add('Open_Box',
										_sm_open_box_5,
										transitions={'finished': 'Warn_Stand_Pose_1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:646 y:365
			OperatableStateMachine.add('Go_To_Stand_Pose_1',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Request_Button_Template', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:642 y:449
			OperatableStateMachine.add('Request_Button_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place Button template"),
										transitions={'received': 'Push_Button', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'template_id'})

			# x:761 y:512
			OperatableStateMachine.add('Push_Button',
										_sm_push_button_4,
										transitions={'finished': 'Warn_Stand_Pose_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side_push', 'template_id': 'template_id', 'grasp_preference': 'grasp_preference'})

			# x:788 y:178
			OperatableStateMachine.add('Warn_Stand_Pose_1',
										LogState(text="Going to STAND pose", severity=Logger.REPORT_INFO),
										transitions={'done': 'Close_Fingers'},
										autonomy={'done': Autonomy.High})

			# x:679 y:599
			OperatableStateMachine.add('Warn_Stand_Pose_2',
										LogState(text="Going to STAND pose", severity=Logger.REPORT_INFO),
										transitions={'done': 'Go_To_Stand_Pose_2'},
										autonomy={'done': Autonomy.High})

			# x:493 y:598
			OperatableStateMachine.add('Go_To_Stand_Pose_2',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:587 y:678
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Step back?", suggestion="walk"),
										transitions={'walk': 'Perform_Step_Back', 'stand': 'finished'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:327 y:672
			OperatableStateMachine.add('Perform_Step_Back',
										_sm_perform_step_back_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'step_back_distance': 'step_back_distance'})

			# x:782 y:278
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1),
										transitions={'done': 'Go_To_Stand_Pose_1', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def calc_turn_back_affordance(self, box_affordance):
		box_affordance.displacement = -box_affordance.displacement * 0.3
		return box_affordance
	
	# [/MANUAL_FUNC]
