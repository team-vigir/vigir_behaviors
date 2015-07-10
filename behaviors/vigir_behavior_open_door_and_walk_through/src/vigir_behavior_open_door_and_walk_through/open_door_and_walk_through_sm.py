#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_open_door_and_walk_through')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.calculation_state import CalculationState
from flexbe_states.log_state import LogState
from flexbe_states.decision_state import DecisionState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_atlas_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_atlas_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_behavior_push_door_open.push_door_open_sm import PushDoorOpenSM
from flexbe_atlas_states.look_at_target_state import LookAtTargetState
from vigir_behavior_grasp_object.grasp_object_sm import GraspObjectSM
from flexbe_atlas_states.get_pose_in_frame_state import GetPoseInFrameState
from flexbe_atlas_states.plan_footsteps_state import PlanFootstepsState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.create_step_goal_state import CreateStepGoalState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Tue Apr 07 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class OpenDoorandWalkThroughSM(Behavior):
	'''
	Performs the open door task challenge.
	'''


	def __init__(self):
		super(OpenDoorandWalkThroughSM, self).__init__()
		self.name = 'Open Door and Walk Through'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('parameter_set', 'drc_step_2D')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk_to_Template')
		self.add_behavior(PushDoorOpenSM, 'Open_Door/Push Door Open')
		self.add_behavior(GraspObjectSM, 'Open_Door/Grasp_Handle')

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

		# x:733 y:390, x:333 y:40
		_sm_hand_back_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_hand_back_0:
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


		# x:1041 y:400, x:987 y:18
		_sm_unlock_door_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['handle_template_id', 'hand_side', 'none', 'door_template_id'], output_keys=['turn_fraction'])

		with _sm_unlock_door_1:
			# x:64 y:28
			OperatableStateMachine.add('Get_Handle_Affordance_Down',
										GetTemplateAffordanceState(identifier=handle_down_affordance),
										transitions={'done': 'Plan_Turn_Handle_Down', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'handle_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

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
										remapping={'template_id': 'handle_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

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
		_sm_release_handle_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none', 'turn_fraction'])

		with _sm_release_handle_2:
			# x:33 y:120
			OperatableStateMachine.add('Decide_Turn_Direction',
										DecisionState(outcomes=["up", "down"], conditions=lambda x: "up" if x > 0 else "down"),
										transitions={'up': 'Get_Handle_Affordance_Up', 'down': 'Get_Handle_Affordance_Down'},
										autonomy={'up': Autonomy.Low, 'down': Autonomy.Low},
										remapping={'input_value': 'turn_fraction'})

			# x:744 y:372
			OperatableStateMachine.add('Hand_Back',
										_sm_hand_back_0,
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
										transitions={'done': 'Open_Fingers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:932 y:228
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


		# x:124 y:577, x:483 y:290
		_sm_traverse_door_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint_distance', 'none'])

		with _sm_traverse_door_3:
			# x:62 y:78
			OperatableStateMachine.add('Generate_Traversing_Waypoint',
										CalculationState(calculation=lambda d: PoseStamped(header=Header(frame_id="pelvis"), pose=Pose(position=Point(x=d), orientation=Quaternion(w=1)))),
										transitions={'done': 'Convert_Waypoint_Frame'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'waypoint_distance', 'output_value': 'waypoint_pelvis'})

			# x:68 y:170
			OperatableStateMachine.add('Convert_Waypoint_Frame',
										GetPoseInFrameState(target_frame='world'),
										transitions={'done': 'Create_Step_Goal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'pose_in': 'waypoint_pelvis', 'pose_out': 'waypoint_world'})

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


		# x:348 y:609, x:119 y:410
		_sm_open_door_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'handle_template_id', 'door_template_id', 'hand_side'])

		with _sm_open_door_4:
			# x:87 y:78
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
										_sm_release_handle_2,
										transitions={'finished': 'Ask_If_Push', 'failed': 'Log_Remove_Hand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'handle_template_id', 'hand_side': 'hand_side', 'none': 'none', 'turn_fraction': 'turn_fraction'})

			# x:777 y:277
			OperatableStateMachine.add('Log_Remove_Hand',
										LogState(text='Remove hand from handle', severity=Logger.REPORT_HINT),
										transitions={'done': 'Ask_If_Push'},
										autonomy={'done': Autonomy.Full})

			# x:440 y:196
			OperatableStateMachine.add('Unlock_Door',
										_sm_unlock_door_1,
										transitions={'finished': 'Ask_for_Retry', 'failed': 'Ask_for_Retry'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'handle_template_id': 'handle_template_id', 'hand_side': 'hand_side', 'none': 'none', 'door_template_id': 'door_template_id', 'turn_fraction': 'turn_fraction'})

			# x:597 y:109
			OperatableStateMachine.add('Ask_for_Retry',
										OperatorDecisionState(outcomes=["release_handle", "retry"], hint="Now release handle?", suggestion="release_handle"),
										transitions={'release_handle': 'Release_Handle', 'retry': 'Log_Try_To_Open'},
										autonomy={'release_handle': Autonomy.High, 'retry': Autonomy.Full})

			# x:587 y:367
			OperatableStateMachine.add('Ask_If_Push',
										OperatorDecisionState(outcomes=['push', 'grasp_again'], hint='Is the door slightly open?', suggestion='push'),
										transitions={'push': 'Look_Straight', 'grasp_again': 'Grasp_Handle'},
										autonomy={'push': Autonomy.High, 'grasp_again': Autonomy.Full})

			# x:283 y:472
			OperatableStateMachine.add('Push Door Open',
										self.use_behavior(PushDoorOpenSM, 'Open_Door/Push Door Open'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:296 y:370
			OperatableStateMachine.add('Look_Straight',
										LookAtTargetState(),
										transitions={'done': 'Push Door Open'},
										autonomy={'done': Autonomy.Off},
										remapping={'frame': 'none'})

			# x:89 y:272
			OperatableStateMachine.add('Grasp_Handle',
										self.use_behavior(GraspObjectSM, 'Open_Door/Grasp_Handle'),
										transitions={'finished': 'Log_Try_To_Open', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'handle_template_id'})

			# x:83 y:178
			OperatableStateMachine.add('Look_At_Handle_Hand',
										LookAtTargetState(),
										transitions={'done': 'Grasp_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'frame': 'template_frame'})



		with _state_machine:
			# x:82 y:28
			OperatableStateMachine.add('Get_Door_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the DOOR template."),
										transitions={'received': 'Manipulate_On', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'door_template_id'})

			# x:616 y:478
			OperatableStateMachine.add('Go_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Traverse_Door', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:81 y:122
			OperatableStateMachine.add('Walk_to_Template',
										self.use_behavior(WalktoTemplateSM, 'Walk_to_Template'),
										transitions={'finished': 'Manipulate_On', 'failed': 'failed', 'aborted': 'Manipulate_On'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'door_template_id'})

			# x:316 y:128
			OperatableStateMachine.add('Manipulate_On',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Align_Door_Log', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:644 y:122
			OperatableStateMachine.add('Open_Door',
										_sm_open_door_4,
										transitions={'finished': 'Wait_For_Gather_Data', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'handle_template_id': 'door_template_id', 'door_template_id': 'door_template_id', 'hand_side': 'hand_side'})

			# x:634 y:278
			OperatableStateMachine.add('Wait_For_Gather_Data',
										LogState(text='Gather data from inside', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_To_Stand'},
										autonomy={'done': Autonomy.Full})

			# x:390 y:472
			OperatableStateMachine.add('Traverse_Door',
										_sm_traverse_door_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint_distance': 'waypoint_distance', 'none': 'none'})

			# x:525 y:34
			OperatableStateMachine.add('Align_Door_Log',
										LogState(text="Adjust pose of the door template", severity=Logger.REPORT_HINT),
										transitions={'done': 'Open_Door'},
										autonomy={'done': Autonomy.Full})


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


	def scale_affordance(self, input_list):
		affordance = input_list[0]
		fraction = input_list[1]
		affordance.displacement *= abs(fraction)
		return affordance
	
	# [/MANUAL_FUNC]
