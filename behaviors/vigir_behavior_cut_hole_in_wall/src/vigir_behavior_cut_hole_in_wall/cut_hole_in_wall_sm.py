#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_cut_hole_in_wall')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_atlas_states.tilt_head_state import TiltHeadState
from flexbe_states.input_state import InputState
from flexbe_atlas_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_atlas_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.get_template_pose_state import GetTemplatePoseState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_behavior_pickup_object.pickup_object_sm import PickupObjectSM
from flexbe_atlas_states.get_template_usability_state import GetTemplateUsabilityState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

import math

# [/MANUAL_IMPORT]


'''
Created on Mon May 11 2015
@author: Spyros Maniatopoulos and Philipp Schillinger
'''
class CutholeinwallSM(Behavior):
	'''
	Behavior that addresses Task 5 of the DRC Finals. Includes walking to and picking up the cutting tool, turning it ON, and using it to cut a circle in the wall.
	'''


	def __init__(self):
		super(CutholeinwallSM, self).__init__()
		self.name = 'Cut hole in wall'

		# parameters of this behavior
		self.add_parameter('hand_side', 'right')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Optionally_Walk_to_Wall/Walk_to_the_Wall')
		self.add_behavior(WalktoTemplateSM, 'Walk_to_and_Pickup_Tool/Walk to Template')
		self.add_behavior(PickupObjectSM, 'Walk_to_and_Pickup_Tool/Pickup Object')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 928 294 /Cut_Circle_in_Wall
		# Complete_Cut SM (up or down)

		# O 928 340 /Cut_Circle_in_Wall
		# Move to center of template and insert



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		cut_angle = 380 # degrees
		insert_distance = 0.05 # meters (used for retract)
		push_distance = 0.08 # meters (for pushing cut piece)
		# x:950 y:178, x:402 y:195
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.tool_template_id = 0
		_state_machine.userdata.none = None
		_state_machine.userdata.center = 'same'
		_state_machine.userdata.tool_side = 'left'
		_state_machine.userdata.wall_side = 'right'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		self._cut_angle = float(cut_angle)
		self._insert_distance = float(abs(insert_distance))
		self._push_distance = float(abs(push_distance))

		# [/MANUAL_CREATE]

		# x:603 y:35, x:95 y:265
		_sm_preparation_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none'], output_keys=['template_id'])

		with _sm_preparation_0:
			# x:327 y:29
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the CUTTING TOOL template."),
										transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:329 y:189
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text="Have no connection to OCS!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:446 y:256
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Received wrong data format!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:730 y:464, x:504 y:59
		_sm_push_inwards_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_push_inwards_1:
			# x:96 y:53
			OperatableStateMachine.add('Get_Insert_Affordance',
										GetTemplateAffordanceState(identifier='insert'),
										transitions={'done': 'Set_Insert_Distance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:101 y:293
			OperatableStateMachine.add('Plan_Insert_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Insert_Affordance', 'incomplete': 'Execute_Insert_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:448 y:293
			OperatableStateMachine.add('Execute_Insert_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Continue_Pushing', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:449 y:455
			OperatableStateMachine.add('Decide_Continue_Pushing',
										OperatorDecisionState(outcomes=['push_more', 'done'], hint='The drill bit must penetrate the wall completely.', suggestion='done'),
										transitions={'push_more': 'Plan_Insert_Affordance', 'done': 'finished'},
										autonomy={'push_more': Autonomy.Full, 'done': Autonomy.Full})

			# x:107 y:167
			OperatableStateMachine.add('Set_Insert_Distance',
										CalculationState(calculation=self.set_push_distance),
										transitions={'done': 'Plan_Insert_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'affordance', 'output_value': 'affordance'})


		# x:30 y:365, x:130 y:365
		_sm_complete_cut_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_complete_cut_2:
			# x:101 y:144
			OperatableStateMachine.add('Placeholder',
										LogState(text='Complete the cut by moving up or down', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})


		# x:917 y:149, x:462 y:284
		_sm_push_cut_piece_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_push_cut_piece_3:
			# x:85 y:53
			OperatableStateMachine.add('Get_Wall_Template_Pose',
										GetTemplatePoseState(),
										transitions={'done': 'Convert_to_List_of_Poses', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'template_id': 'template_id', 'template_pose': 'template_pose'})

			# x:67 y:386
			OperatableStateMachine.add('Plan_to_Wall_Center',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=False, include_torso=False, keep_endeffector_orientation=True, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Go_to_Wall_Center', 'incomplete': 'Go_to_Wall_Center', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'waypoints': 'waypoints', 'hand': 'hand_side', 'frame_id': 'frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:84 y:160
			OperatableStateMachine.add('Convert_to_List_of_Poses',
										CalculationState(calculation=lambda pose: [pose.pose]),
										transitions={'done': 'Prepare_Plan_Frame_Wrist'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'template_pose', 'output_value': 'waypoints'})

			# x:86 y:523
			OperatableStateMachine.add('Go_to_Wall_Center',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Push_Inwards', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:83 y:268
			OperatableStateMachine.add('Prepare_Plan_Frame_Wrist',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Plan_to_Wall_Center'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'template_pose', 'output_value': 'frame_id'})

			# x:591 y:515
			OperatableStateMachine.add('Push_Inwards',
										_sm_push_inwards_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})


		# x:709 y:397, x:504 y:96
		_sm_insert_drill_bit_in_wall_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_insert_drill_bit_in_wall_4:
			# x:96 y:90
			OperatableStateMachine.add('Get_Insert_Affordance',
										GetTemplateAffordanceState(identifier='insert'),
										transitions={'done': 'Plan_Insert_Affordance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:102 y:248
			OperatableStateMachine.add('Plan_Insert_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Insert_Affordance', 'incomplete': 'Execute_Insert_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:448 y:247
			OperatableStateMachine.add('Execute_Insert_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Continue_Insertion', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:449 y:388
			OperatableStateMachine.add('Decide_Continue_Insertion',
										OperatorDecisionState(outcomes=['insert_more', 'done'], hint='The drill bit must penetrate the wall completely.', suggestion='done'),
										transitions={'insert_more': 'Plan_Insert_Affordance', 'done': 'finished'},
										autonomy={'insert_more': Autonomy.Full, 'done': Autonomy.Full})


		# x:806 y:460, x:350 y:132
		_sm_retract_drill_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_retract_drill_5:
			# x:33 y:61
			OperatableStateMachine.add('Get_Insert_Affordance',
										GetTemplateAffordanceState(identifier='insert'),
										transitions={'done': 'Set_Retract_Distance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:36 y:335
			OperatableStateMachine.add('Plan_Retract_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Retract_Affordance', 'incomplete': 'Execute_Retract_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:532 y:336
			OperatableStateMachine.add('Execute_Retract_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Continue_Extraction', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:41 y:199
			OperatableStateMachine.add('Set_Retract_Distance',
										CalculationState(calculation=self.set_retract_distance),
										transitions={'done': 'Plan_Retract_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'affordance', 'output_value': 'affordance'})

			# x:533 y:450
			OperatableStateMachine.add('Decide_Continue_Extraction',
										OperatorDecisionState(outcomes=['extract_more', 'done'], hint='The drill must exit completely.', suggestion='done'),
										transitions={'extract_more': 'Plan_Retract_Affordance', 'done': 'finished'},
										autonomy={'extract_more': Autonomy.Full, 'done': Autonomy.Full})


		# x:953 y:206, x:366 y:223
		_sm_cut_circle_in_wall_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'drill_usability'])

		with _sm_cut_circle_in_wall_6:
			# x:59 y:56
			OperatableStateMachine.add('Get_Cut_Circle_Affordance',
										GetTemplateAffordanceState(identifier='cut_circle'),
										transitions={'done': 'Set_Cut_Circle_Angle', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:540 y:57
			OperatableStateMachine.add('Plan_Cut_Circle_Affordance',
										PlanAffordanceState(vel_scaling=0.03, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Cut_Circle_Affordance', 'incomplete': 'Execute_Cut_Circle_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'drill_usability', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:533 y:340
			OperatableStateMachine.add('Execute_Cut_Circle_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Done_Cutting', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:317 y:57
			OperatableStateMachine.add('Set_Cut_Circle_Angle',
										CalculationState(calculation=self.set_cut_circle_angle),
										transitions={'done': 'Plan_Cut_Circle_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'affordance', 'output_value': 'affordance'})

			# x:698 y:199
			OperatableStateMachine.add('Decide_Done_Cutting',
										OperatorDecisionState(outcomes=['cut_more', 'done'], hint='Keep going until a full circle has been cut.', suggestion='cut_more'),
										transitions={'cut_more': 'Plan_Cut_Circle_Affordance', 'done': 'finished'},
										autonomy={'cut_more': Autonomy.High, 'done': Autonomy.High})


		# x:860 y:82, x:360 y:283
		_sm_hand_in_front_of_wall_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'preference'])

		with _sm_hand_in_front_of_wall_7:
			# x:55 y:77
			OperatableStateMachine.add('Get_Wall_Pregrasp_Pose',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_to_Wall_Pregrasp', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'preference', 'pre_grasp': 'pre_grasp'})

			# x:579 y:77
			OperatableStateMachine.add('Go_to_Wall_Pregrasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:316 y:79
			OperatableStateMachine.add('Plan_to_Wall_Pregrasp',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Go_to_Wall_Pregrasp', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_pose': 'pre_grasp', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})


		# x:788 y:612, x:67 y:501
		_sm_walk_to_and_pickup_tool_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id', 'none', 'tool_side'], output_keys=['drill_usability'])

		with _sm_walk_to_and_pickup_tool_8:
			# x:44 y:72
			OperatableStateMachine.add('Preparation',
										_sm_preparation_0,
										transitions={'finished': 'Ask_Perform_Walking', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'template_id': 'template_id'})

			# x:284 y:78
			OperatableStateMachine.add('Ask_Perform_Walking',
										OperatorDecisionState(outcomes=['walk', 'stand'], hint="Does the robot need to walk to the table?", suggestion='walk'),
										transitions={'walk': 'Walk to Template', 'stand': 'Set_Manipulate'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:531 y:22
			OperatableStateMachine.add('Walk to Template',
										self.use_behavior(WalktoTemplateSM, 'Walk_to_and_Pickup_Tool/Walk to Template'),
										transitions={'finished': 'Set_Manipulate', 'failed': 'Walk_Manually', 'aborted': 'Walk_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:769 y:82
			OperatableStateMachine.add('Walk_Manually',
										LogState(text="Guide the robot to the template manually.", severity=Logger.REPORT_HINT),
										transitions={'done': 'Set_Manipulate'},
										autonomy={'done': Autonomy.Full})

			# x:541 y:412
			OperatableStateMachine.add('Pickup Object',
										self.use_behavior(PickupObjectSM, 'Walk_to_and_Pickup_Tool/Pickup Object'),
										transitions={'finished': 'Optional_Template_Alignment', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:546 y:323
			OperatableStateMachine.add('Head_Look_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_30),
										transitions={'done': 'Pickup Object', 'failed': 'Head_Look_Down'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full})

			# x:518 y:144
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Turn_Torso', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Full})

			# x:528 y:235
			OperatableStateMachine.add('Turn_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_FULL, vel_scaling=0.2, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Head_Look_Down', 'failed': 'Turn_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'tool_side'})

			# x:532 y:603
			OperatableStateMachine.add('Get_Drill_bit_Usability',
										GetTemplateUsabilityState(usability_name='bit', usability_id=100),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'template_id': 'template_id', 'usability_pose': 'drill_usability'})

			# x:519 y:513
			OperatableStateMachine.add('Optional_Template_Alignment',
										LogState(text='Request attached template alignment', severity=Logger.REPORT_HINT),
										transitions={'done': 'Get_Drill_bit_Usability'},
										autonomy={'done': Autonomy.High})


		# x:458 y:49, x:460 y:203
		_sm_optionally_walk_to_wall_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['wall_template_id', 'grasp_preference', 'hand_side', 'center'])

		with _sm_optionally_walk_to_wall_9:
			# x:67 y:43
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=['walk', 'stay'], hint='Decide whether to walk to the wall or just stay here.', suggestion='stay'),
										transitions={'walk': 'Retract_Arms', 'stay': 'finished'},
										autonomy={'walk': Autonomy.Full, 'stay': Autonomy.High})

			# x:642 y:338
			OperatableStateMachine.add('Go_to_STAND_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND_MANIPULATE),
										transitions={'changed': 'Walk_to_the_Wall', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Full})

			# x:658 y:43
			OperatableStateMachine.add('Walk_to_the_Wall',
										self.use_behavior(WalktoTemplateSM, 'Optionally_Walk_to_Wall/Walk_to_the_Wall'),
										transitions={'finished': 'finished', 'failed': 'failed', 'aborted': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'wall_template_id'})

			# x:61 y:195
			OperatableStateMachine.add('Retract_Arms',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.BOTH_ARMS_SIDES, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Center_Torso', 'failed': 'Retract_Arms'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'center'})

			# x:62 y:333
			OperatableStateMachine.add('Center_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_CENTER_POSE, vel_scaling=0.2, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Go_to_STAND_MANIPULATE', 'failed': 'Center_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'center'})


		# x:176 y:449, x:303 y:229
		_sm_cut_circle_in_wall_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'preference', 'none', 'drill_usability'])

		with _sm_cut_circle_in_wall_10:
			# x:90 y:54
			OperatableStateMachine.add('Hand_in_front_of_Wall',
										_sm_hand_in_front_of_wall_7,
										transitions={'finished': 'Insert_Drill_bit_in_Wall', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'preference'})

			# x:774 y:56
			OperatableStateMachine.add('Cut_Circle_in_Wall',
										_sm_cut_circle_in_wall_6,
										transitions={'finished': 'Retract_Drill', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'drill_usability': 'drill_usability'})

			# x:791 y:213
			OperatableStateMachine.add('Retract_Drill',
										_sm_retract_drill_5,
										transitions={'finished': 'Complete_Cut', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:458 y:224
			OperatableStateMachine.add('Retry_Cutting',
										OperatorDecisionState(outcomes=['fail', 'pregrasp', 'insert', 'cut'], hint='Decide if and how to retry', suggestion=None),
										transitions={'fail': 'failed', 'pregrasp': 'Hand_in_front_of_Wall', 'insert': 'Insert_Drill_bit_in_Wall', 'cut': 'Cut_Circle_in_Wall'},
										autonomy={'fail': Autonomy.Full, 'pregrasp': Autonomy.High, 'insert': Autonomy.High, 'cut': Autonomy.High})

			# x:445 y:56
			OperatableStateMachine.add('Insert_Drill_bit_in_Wall',
										_sm_insert_drill_bit_in_wall_4,
										transitions={'finished': 'Cut_Circle_in_Wall', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:457 y:436
			OperatableStateMachine.add('Push_Cut_Piece',
										_sm_push_cut_piece_3,
										transitions={'finished': 'finished', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:788 y:437
			OperatableStateMachine.add('Complete_Cut',
										_sm_complete_cut_2,
										transitions={'finished': 'Push_Cut_Piece', 'failed': 'Retry_Cutting'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:651 y:83, x:392 y:405
		_sm_request_wall_template_11 = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['template_id'])

		with _sm_request_wall_template_11:
			# x:80 y:80
			OperatableStateMachine.add('Head_Look_Straight',
										TiltHeadState(desired_tilt=TiltHeadState.STRAIGHT),
										transitions={'done': 'Request_Template_ID', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High})

			# x:341 y:81
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the WALL template."),
										transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:606 y:198
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text="Have no connection to OCS!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:445 y:192
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Received wrong data format!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})



		with _state_machine:
			# x:72 y:28
			OperatableStateMachine.add('Notify_Execution_Started',
										LogState(text='Execution has started. Consider making modifications if this is a re-run.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Walk_to_and_Pickup_Tool'},
										autonomy={'done': Autonomy.Full})

			# x:890 y:59
			OperatableStateMachine.add('Notify_Behavior_Exit',
										LogState(text='The behavior will now exit. Last chance to intervene!', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})

			# x:65 y:452
			OperatableStateMachine.add('Request_Wall_Template',
										_sm_request_wall_template_11,
										transitions={'finished': 'Set_Manipulate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'wall_template_id'})

			# x:634 y:55
			OperatableStateMachine.add('Cut_Circle_in_Wall',
										_sm_cut_circle_in_wall_10,
										transitions={'finished': 'Notify_Behavior_Exit', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'wall_template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'none': 'none', 'drill_usability': 'drill_usability'})

			# x:621 y:322
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Turn_Torso', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Full})

			# x:621 y:455
			OperatableStateMachine.add('Optionally_Walk_to_Wall',
										_sm_optionally_walk_to_wall_9,
										transitions={'finished': 'Set_Manipulate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'wall_template_id': 'wall_template_id', 'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'center': 'center'})

			# x:63 y:184
			OperatableStateMachine.add('Walk_to_and_Pickup_Tool',
										_sm_walk_to_and_pickup_tool_8,
										transitions={'finished': 'Request_Wall_Template', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'tool_template_id', 'none': 'none', 'tool_side': 'tool_side', 'drill_usability': 'drill_usability'})

			# x:630 y:197
			OperatableStateMachine.add('Turn_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_SLIGHTLY, vel_scaling=0.2, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Cut_Circle_in_Wall', 'failed': 'Turn_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'wall_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def set_cut_circle_angle(self, affordance):
		'''Sets the angle of the circular affordance for cutting a circle.'''

		cut_angle_degrees = math.copysign(self._cut_angle, affordance.displacement)
		affordance.displacement = math.radians(cut_angle_degrees)
		Logger.loginfo("Will be cutting by %d degrees" % cut_angle_degrees)
		
		return affordance

	def set_retract_distance(self, affordance):
		'''Sets the displacement of the "insert" affordance to a negative number.'''

		retract_distance = -abs(self._insert_distance) # negative number means retract
		affordance.displacement = retract_distance
		Logger.loginfo("Will be retracting by %d centimeters" % (retract_distance*100))
		
		return affordance

	def set_push_distance(self, affordance):
		'''Sets the displacement of the "insert" affordance to a positive number.'''

		push_distance = +abs(self._push_distance)
		affordance.displacement = push_distance
		Logger.loginfo("Will be retracting by %d centimeters" % (push_distance*100))
		
		return affordance

	# [/MANUAL_FUNC]
