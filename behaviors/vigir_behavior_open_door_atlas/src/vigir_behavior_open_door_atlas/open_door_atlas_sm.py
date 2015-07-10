#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_open_door_atlas')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_atlas_states.tilt_head_state import TiltHeadState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_states.log_state import LogState
from flexbe_atlas_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_states.calculation_state import CalculationState
from flexbe_atlas_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from flexbe_atlas_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 18 2015
@author: Spyros Maniatopoulos
'''
class OpenDoorATLASSM(Behavior):
	'''
	A behavior for performing Task 3 with ATLAS.
	'''


	def __init__(self):
		super(OpenDoorATLASSM, self).__init__()
		self.name = 'Open Door ATLAS'

		# parameters of this behavior
		self.add_parameter('hand_side', 'right')
		self.add_parameter('hand_type', 'robotiq')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk_to_Template')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		turn_handle_affordance = 'Up'
		strafe_direction = FootstepPlanRelativeState.DIRECTION_RIGHT if self.hand_side == 'right' else FootstepPlanRelativeState.DIRECTION_LEFT
		# x:16 y:476, x:360 y:313
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.zero = 0
		_state_machine.userdata.both_arms = 'same'
		_state_machine.userdata.strafe_distance = 3.0 # meters
		_state_machine.userdata.stand_pose_preference = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:851 y:272, x:198 y:293
		_sm_push_door_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['handle_template_id', 'hand_side', 'none'])

		with _sm_push_door_0:
			# x:156 y:74
			OperatableStateMachine.add('Get_Push_Affordance',
										GetTemplateAffordanceState(identifier='push'),
										transitions={'done': 'Plan_Push_Door', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'handle_template_id', 'hand_side': 'hand_side', 'affordance': 'door_affordance'})

			# x:781 y:72
			OperatableStateMachine.add('Push_Door',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:465 y:73
			OperatableStateMachine.add('Plan_Push_Door',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Push_Door', 'incomplete': 'Push_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'door_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})


		# x:642 y:277, x:123 y:475
		_sm_turn_handle_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['handle_template_id', 'hand_side', 'none'])

		with _sm_turn_handle_1:
			# x:73 y:78
			OperatableStateMachine.add('Get_Handle_Affordance',
										GetTemplateAffordanceState(identifier=turn_handle_affordance),
										transitions={'done': 'Plan_Turn_Handle', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'handle_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:342 y:78
			OperatableStateMachine.add('Plan_Turn_Handle',
										PlanAffordanceState(vel_scaling=0.03, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Turn_Handle', 'incomplete': 'Turn_Handle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:580 y:77
			OperatableStateMachine.add('Turn_Handle',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})


		# x:1075 y:57, x:260 y:86, x:1048 y:173
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


		# x:30 y:365, x:130 y:365
		_sm_go_to_pregrasp_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose'])

		with _sm_go_to_pregrasp_3:
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


		# x:372 y:343, x:567 y:184
		_sm_unlock_door_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['handle_template_id', 'hand_side', 'none'])

		with _sm_unlock_door_4:
			# x:75 y:64
			OperatableStateMachine.add('Turn_Handle',
										_sm_turn_handle_1,
										transitions={'finished': 'Decide_Push_Turn', 'failed': 'Unlock_Door_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'handle_template_id': 'handle_template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:328 y:178
			OperatableStateMachine.add('Unlock_Door_Manually',
										OperatorDecisionState(outcomes=["ready", "abort"], hint='Let the operator open the door manually.', suggestion=None),
										transitions={'ready': 'finished', 'abort': 'failed'},
										autonomy={'ready': Autonomy.Full, 'abort': Autonomy.Full})

			# x:69 y:336
			OperatableStateMachine.add('Decide_Push_Turn',
										OperatorDecisionState(outcomes=['push', 'turn', 'skip'], hint='Ask operator whether to push more or turn again.', suggestion='skip'),
										transitions={'push': 'Push_Door', 'turn': 'Turn_Handle', 'skip': 'finished'},
										autonomy={'push': Autonomy.High, 'turn': Autonomy.High, 'skip': Autonomy.Full})

			# x:72 y:485
			OperatableStateMachine.add('Push_Door',
										_sm_push_door_0,
										transitions={'finished': 'Decide_Push_Turn', 'failed': 'Push_Door'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'handle_template_id': 'handle_template_id', 'hand_side': 'hand_side', 'none': 'none'})


		# x:820 y:178
		_sm_traverse_door_5 = OperatableStateMachine(outcomes=['finished'], input_keys=['strafe_distance'])

		with _sm_traverse_door_5:
			# x:59 y:67
			OperatableStateMachine.add('Plan_Strafing',
										FootstepPlanRelativeState(direction=strafe_direction),
										transitions={'planned': 'Confirm_Plan', 'failed': 'Request_Adjustment'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'distance': 'strafe_distance', 'plan_header': 'plan_header'})

			# x:64 y:228
			OperatableStateMachine.add('Request_Adjustment',
										LogState(text='Ask the operator to align the robot with the door.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Plan_Strafing'},
										autonomy={'done': Autonomy.Full})

			# x:583 y:65
			OperatableStateMachine.add('Strafe_through_Door',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Notify_Strafe_End', 'failed': 'Request_Adjustment'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'plan_header': 'plan_header'})

			# x:318 y:66
			OperatableStateMachine.add('Confirm_Plan',
										OperatorDecisionState(outcomes=['ready', 'adjust'], hint='Check if the robot is perpendicular to the door.', suggestion=None),
										transitions={'ready': 'Strafe_through_Door', 'adjust': 'Request_Adjustment'},
										autonomy={'ready': Autonomy.Full, 'adjust': Autonomy.Low})

			# x:603 y:173
			OperatableStateMachine.add('Notify_Strafe_End',
										LogState(text='End of strafing. Switching to MANIPULATE next!', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})


		# x:484 y:37, x:383 y:171
		_sm_open_door_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'hand_side', 'grasp_preference', 'handle_template_id'])

		with _sm_open_door_6:
			# x:69 y:28
			OperatableStateMachine.add('Look_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_45),
										transitions={'done': 'Turn_Torso', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High})

			# x:643 y:451
			OperatableStateMachine.add('Unlock_Door',
										_sm_unlock_door_4,
										transitions={'finished': 'Ask_for_Retry', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'handle_template_id': 'handle_template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:636 y:248
			OperatableStateMachine.add('Ask_for_Retry',
										OperatorDecisionState(outcomes=["done", "retry"], hint="Now release handle?", suggestion="done"),
										transitions={'done': 'Decide_Push_or_Not', 'retry': 'Optional_Template_Adjustment'},
										autonomy={'done': Autonomy.High, 'retry': Autonomy.Full})

			# x:51 y:134
			OperatableStateMachine.add('Turn_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_FULL, vel_scaling=0.2, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Turn_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:625 y:34
			OperatableStateMachine.add('Push_Door_with_Arm',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_OPEN_POSE_SIDE, vel_scaling=0.3, ignore_collisions=True, link_paddings={}, is_cartesian=True),
										transitions={'done': 'finished', 'failed': 'Push_Door_with_Arm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:44 y:246
			OperatableStateMachine.add('Optional_Template_Adjustment',
										LogState(text='Ask the operator to adjust the template, if necessary.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Confirm_Closed_Fingers'},
										autonomy={'done': Autonomy.High})

			# x:59 y:451
			OperatableStateMachine.add('Go_to_Pregrasp',
										_sm_go_to_pregrasp_3,
										transitions={'finished': 'Go_to_Grasp', 'failed': 'Ask_for_Retry'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'handle_template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:348 y:451
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_2,
										transitions={'finished': 'Unlock_Door', 'failed': 'failed', 'again': 'Go_to_Pregrasp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'handle_template_id'})

			# x:54 y:352
			OperatableStateMachine.add('Confirm_Closed_Fingers',
										LogState(text='Check that the fingers are closed!', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_to_Pregrasp'},
										autonomy={'done': Autonomy.High})

			# x:637 y:137
			OperatableStateMachine.add('Decide_Push_or_Not',
										OperatorDecisionState(outcomes=['push', 'skip'], hint='Check whether the door is already open.', suggestion=None),
										transitions={'push': 'Push_Door_with_Arm', 'skip': 'finished'},
										autonomy={'push': Autonomy.High, 'skip': Autonomy.High})



		with _state_machine:
			# x:73 y:310
			OperatableStateMachine.add('Get_Door_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the DOOR."),
										transitions={'received': 'Start_in_STAND', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.Low, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:730 y:362
			OperatableStateMachine.add('Go_To_Stand_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND_MANIPULATE),
										transitions={'changed': 'Tilt_Head_Straight', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:72 y:21
			OperatableStateMachine.add('Walk_to_Template',
										self.use_behavior(WalktoTemplateSM, 'Walk_to_Template'),
										transitions={'finished': 'Go_to_MANIPULATE', 'failed': 'failed', 'aborted': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'stand_pose_preference', 'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:303 y:25
			OperatableStateMachine.add('Go_to_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Open_or_Traverse', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:761 y:24
			OperatableStateMachine.add('Open_Door',
										_sm_open_door_6,
										transitions={'finished': 'Move_Arms_Sides', 'failed': 'Open_Door_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'handle_template_id': 'template_id'})

			# x:758 y:591
			OperatableStateMachine.add('Traverse_Door',
										_sm_traverse_door_5,
										transitions={'finished': 'Go_back_to_MANIPULATE'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'strafe_distance': 'strafe_distance'})

			# x:996 y:78
			OperatableStateMachine.add('Open_Door_Manually',
										LogState(text='Have the operator open the door manually!', severity=Logger.REPORT_HINT),
										transitions={'done': 'Move_Arms_Sides'},
										autonomy={'done': Autonomy.Full})

			# x:762 y:479
			OperatableStateMachine.add('Tilt_Head_Straight',
										TiltHeadState(desired_tilt=TiltHeadState.STRAIGHT),
										transitions={'done': 'Traverse_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High})

			# x:739 y:156
			OperatableStateMachine.add('Move_Arms_Sides',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.BOTH_ARMS_SIDES, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Center_Torso', 'failed': 'Move_Arms_Sides'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'both_arms'})

			# x:432 y:600
			OperatableStateMachine.add('Go_back_to_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Move_to_Stand_Pose', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:167 y:601
			OperatableStateMachine.add('Move_to_Stand_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'End_in_STAND', 'failed': 'Move_to_Stand_Pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'both_arms'})

			# x:160 y:468
			OperatableStateMachine.add('End_in_STAND',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:741 y:266
			OperatableStateMachine.add('Center_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_CENTER_POSE, vel_scaling=0.1, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Go_To_Stand_Manipulate', 'failed': 'Center_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'both_arms'})

			# x:63 y:179
			OperatableStateMachine.add('Start_in_STAND',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Walk_to_Template', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:555 y:27
			OperatableStateMachine.add('Open_or_Traverse',
										OperatorDecisionState(outcomes=['open', 'traverse'], hint='If this is after a reset, the door will be open.', suggestion='open'),
										transitions={'open': 'Open_Door', 'traverse': 'Move_Arms_Sides'},
										autonomy={'open': Autonomy.Low, 'traverse': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
