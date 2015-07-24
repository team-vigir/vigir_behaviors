#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_surprise_plug')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_behavior_grasp_object.grasp_object_sm import GraspObjectSM
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_states.calculation_state import CalculationState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from vigir_flexbe_states.get_template_grasp_state import GetTemplateGraspState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from vigir_flexbe_states.detach_object_state import DetachObjectState
from vigir_flexbe_states.attach_object_state import AttachObjectState
from flexbe_states.log_state import LogState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.footstep_plan_relative_state import FootstepPlanRelativeState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 01 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class SurprisePlugSM(Behavior):
	'''
	do the plug surprise task
	'''


	def __init__(self):
		super(SurprisePlugSM, self).__init__()
		self.name = 'Surprise Plug'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk_To_Template')
		self.add_behavior(GraspObjectSM, 'Grasp Object')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		plug_in_affordance = "plug_in"
		plug_out_affordance = "plug_out"
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		# x:202 y:568, x:374 y:401
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.none = None
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.step_back_distance = 1.0 # meters

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:588 y:126, x:324 y:44
		_sm_back_to_pregrasp_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'grasp_preference'])

		with _sm_back_to_pregrasp_0:
			# x:30 y:40
			OperatableStateMachine.add('Get_Pregrasp',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'grasp_pose'})

			# x:242 y:292
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'Get_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:568 y:295
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Get_Pregrasp'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})

			# x:40 y:293
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})


		# x:598 y:30, x:220 y:148, x:1035 y:174
		_sm_go_to_grasp_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference'])

		with _sm_go_to_grasp_1:
			# x:33 y:49
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
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
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Decide_Which_Grasp'},
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
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:596 y:113, x:351 y:62
		_sm_go_to_pregrasp_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose'])

		with _sm_go_to_pregrasp_2:
			# x:27 y:68
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:537 y:228
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

			# x:266 y:228
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:266 y:327
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Pregrasp_Info', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.Full})


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


		# x:550 y:574, x:130 y:365
		_sm_plug_in_cable_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_template_id', 'hand_side', 'grasp_preference', 'template_id', 'template_pose'])

		with _sm_plug_in_cable_4:
			# x:82 y:59
			OperatableStateMachine.add('Go_to_Pregrasp',
										_sm_go_to_pregrasp_2,
										transitions={'finished': 'Go_to_Grasp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'target_template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:322 y:171
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_1,
										transitions={'finished': 'Detach_Cable', 'failed': 'failed', 'again': 'Go_to_Pregrasp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'target_template_id'})

			# x:516 y:306
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'Back_To_Pregrasp', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:472 y:406
			OperatableStateMachine.add('Back_To_Pregrasp',
										_sm_back_to_pregrasp_0,
										transitions={'finished': 'Close_Fingers', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference'})

			# x:314 y:524
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:527 y:204
			OperatableStateMachine.add('Detach_Cable',
										DetachObjectState(),
										transitions={'done': 'Open_Fingers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'template_id': 'template_id', 'template_pose': 'template_pose'})


		# x:30 y:365, x:115 y:197
		_sm_take_cable_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_take_cable_5:
			# x:81 y:54
			OperatableStateMachine.add('Get_Plug_Out_Affordance',
										GetTemplateAffordanceState(identifier=plug_out_affordance),
										transitions={'done': 'Plan_Plug_Out', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:348 y:73
			OperatableStateMachine.add('Plan_Plug_Out',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Plug_Out', 'incomplete': 'Execute_Plug_Out', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:322 y:255
			OperatableStateMachine.add('Execute_Plug_Out',
										ExecuteTrajectoryMsgState(controller=affordance_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})



		with _state_machine:
			# x:73 y:78
			OperatableStateMachine.add('Request_Cable_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place cable template"),
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

			# x:839 y:72
			OperatableStateMachine.add('Grasp Object',
										self.use_behavior(GraspObjectSM, 'Grasp Object'),
										transitions={'finished': 'Attach_Cable', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:843 y:302
			OperatableStateMachine.add('Take_Cable',
										_sm_take_cable_5,
										transitions={'finished': 'Request_Target_Template', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:566 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Grasp Object', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:822 y:545
			OperatableStateMachine.add('Plug_In_Cable',
										_sm_plug_in_cable_4,
										transitions={'finished': 'Warn_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_template_id': 'target_template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'template_pose': 'template_pose'})

			# x:826 y:185
			OperatableStateMachine.add('Attach_Cable',
										AttachObjectState(),
										transitions={'done': 'Take_Cable', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'template_pose': 'template_pose'})

			# x:811 y:400
			OperatableStateMachine.add('Request_Target_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place target template"),
										transitions={'received': 'Plug_In_Cable', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.High, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'target_template_id'})

			# x:753 y:646
			OperatableStateMachine.add('Warn_Stand',
										LogState(text="Going to stand pose", severity=Logger.REPORT_INFO),
										transitions={'done': 'Go_To_Stand_Pose'},
										autonomy={'done': Autonomy.High})

			# x:581 y:573
			OperatableStateMachine.add('Go_To_Stand_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Set_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:446 y:653
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Step back?", suggestion="walk"),
										transitions={'walk': 'Perform_Step_Back', 'stand': 'finished'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:100 y:661
			OperatableStateMachine.add('Perform_Step_Back',
										_sm_perform_step_back_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'step_back_distance': 'step_back_distance'})

			# x:383 y:527
			OperatableStateMachine.add('Set_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
