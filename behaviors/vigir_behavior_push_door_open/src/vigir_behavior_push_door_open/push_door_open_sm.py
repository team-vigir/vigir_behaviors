#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_push_door_open')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_states.log_state import LogState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri May 15 2015
@author: Dorothea Koert, Philipp Schillinger
'''
class PushDoorOpenSM(Behavior):
	'''
	Performs a series of motions to push against the unlocked door.
	'''


	def __init__(self):
		super(PushDoorOpenSM, self).__init__()
		self.name = 'Push Door Open'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:923 y:114
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.none = None
		_state_machine.userdata.do_turn_torso = False
		_state_machine.userdata.pushing_side = 'right' if self.hand_side == 'left' else 'left'
		_state_machine.userdata.torso_side = self.hand_side

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:333 y:440, x:433 y:240
		_sm_opening_motion_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['do_turn_torso', 'pushing_side', 'none', 'torso_side'])

		with _sm_opening_motion_0:
			# x:438 y:28
			OperatableStateMachine.add('Branch_Torso_Open',
										DecisionState(outcomes=['turn', 'fixed'], conditions=lambda x: 'turn' if x else 'fixed'),
										transitions={'turn': 'Turn_Torso', 'fixed': 'Go_To_Open_Door_Pose_Straight'},
										autonomy={'turn': Autonomy.Low, 'fixed': Autonomy.Low},
										remapping={'input_value': 'do_turn_torso'})

			# x:57 y:278
			OperatableStateMachine.add('Go_To_Open_Door_Pose_Turned',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_OPEN_POSE_TURNED, vel_scaling=0.05, ignore_collisions=True, link_paddings={}),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'pushing_side'})

			# x:705 y:228
			OperatableStateMachine.add('Go_To_Open_Door_Pose_Straight',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_OPEN_POSE_STRAIGHT, vel_scaling=0.05, ignore_collisions=True, link_paddings={}),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'pushing_side'})

			# x:76 y:128
			OperatableStateMachine.add('Turn_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_FULL, vel_scaling=0.05, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Go_To_Open_Door_Pose_Turned', 'failed': 'Log_No_Turn'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'torso_side'})

			# x:484 y:128
			OperatableStateMachine.add('Set_Turn_Torso_False',
										CalculationState(calculation=lambda x: False),
										transitions={'done': 'Go_To_Open_Door_Pose_Straight'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'do_turn_torso'})

			# x:305 y:128
			OperatableStateMachine.add('Log_No_Turn',
										LogState(text="Skip turning because of collision", severity=Logger.REPORT_INFO),
										transitions={'done': 'Set_Turn_Torso_False'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:287 y:78
			OperatableStateMachine.add('Decide_If_Turn',
										OperatorDecisionState(outcomes=['turn_torso', 'fixed_torso'], hint=None, suggestion=None),
										transitions={'turn_torso': 'Decide_Go_To_Stand', 'fixed_torso': 'Go_To_Door_Ready_Pose'},
										autonomy={'turn_torso': Autonomy.Full, 'fixed_torso': Autonomy.Full})

			# x:125 y:478
			OperatableStateMachine.add('Go_To_Door_Ready_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_READY_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Opening_Motion', 'failed': 'Decide_No_Collision_Avoidance'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'pushing_side'})

			# x:887 y:378
			OperatableStateMachine.add('Ask_If_Open',
										OperatorDecisionState(outcomes=['open','push_again'], hint='Is the door open?', suggestion='open'),
										transitions={'open': 'Go_To_Final_Stand', 'push_again': 'Log_Try_Push_Again'},
										autonomy={'open': Autonomy.High, 'push_again': Autonomy.Full})

			# x:521 y:82
			OperatableStateMachine.add('Log_Try_Push_Again',
										LogState(text='Move robot closer to door', severity=Logger.REPORT_HINT),
										transitions={'done': 'Decide_If_Turn'},
										autonomy={'done': Autonomy.Full})

			# x:620 y:628
			OperatableStateMachine.add('Back_To_Door_Ready_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_READY_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Branch_Torso_Retract', 'failed': 'Open_Manually'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'pushing_side'})

			# x:136 y:328
			OperatableStateMachine.add('Set_Turn_Torso_True',
										CalculationState(calculation=lambda x: True),
										transitions={'done': 'Go_To_Door_Ready_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'do_turn_torso'})

			# x:186 y:622
			OperatableStateMachine.add('Opening_Motion',
										_sm_opening_motion_0,
										transitions={'finished': 'Back_To_Door_Ready_Pose', 'failed': 'Open_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'do_turn_torso': 'do_turn_torso', 'pushing_side': 'pushing_side', 'none': 'none', 'torso_side': 'torso_side'})

			# x:893 y:201
			OperatableStateMachine.add('Go_To_Final_Stand',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'none'})

			# x:826 y:528
			OperatableStateMachine.add('Turn_Torso_Center_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_CENTER_POSE, vel_scaling=0.05, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Ask_If_Open', 'failed': 'Open_Manually'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'none'})

			# x:1034 y:628
			OperatableStateMachine.add('Branch_Torso_Retract',
										DecisionState(outcomes=['turn', 'fixed'], conditions=lambda x: 'turn' if x else 'fixed'),
										transitions={'turn': 'Turn_Torso_Center_Pose', 'fixed': 'Ask_If_Open'},
										autonomy={'turn': Autonomy.Low, 'fixed': Autonomy.Low},
										remapping={'input_value': 'do_turn_torso'})

			# x:26 y:278
			OperatableStateMachine.add('Go_To_Stand',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Set_Turn_Torso_True', 'failed': 'Go_To_Door_Ready_Pose'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'none'})

			# x:136 y:128
			OperatableStateMachine.add('Decide_Go_To_Stand',
										DecisionState(outcomes=['stand', 'skip'], conditions=lambda x: 'stand' if not x else 'skip'),
										transitions={'stand': 'Check_Hand_Space', 'skip': 'Set_Turn_Torso_True'},
										autonomy={'stand': Autonomy.Low, 'skip': Autonomy.Low},
										remapping={'input_value': 'do_turn_torso'})

			# x:360 y:378
			OperatableStateMachine.add('Decide_No_Collision_Avoidance',
										OperatorDecisionState(outcomes=['replan_ignore_collisions', 'continue', 'open_manual'], hint="Try again and ignore collisions?", suggestion='replan_ignore_collisions'),
										transitions={'replan_ignore_collisions': 'Go_To_Door_Ready_Pose_NC', 'continue': 'Opening_Motion', 'open_manual': 'Open_Manually'},
										autonomy={'replan_ignore_collisions': Autonomy.Full, 'continue': Autonomy.Full, 'open_manual': Autonomy.Full})

			# x:365 y:528
			OperatableStateMachine.add('Go_To_Door_Ready_Pose_NC',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_READY_POSE, vel_scaling=0.3, ignore_collisions=True, link_paddings={}),
										transitions={'done': 'Opening_Motion', 'failed': 'Open_Manually'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'pushing_side'})

			# x:651 y:528
			OperatableStateMachine.add('Open_Manually',
										LogState(text="Open door manually", severity=Logger.REPORT_HINT),
										transitions={'done': 'Ask_If_Open'},
										autonomy={'done': Autonomy.Full})

			# x:40 y:178
			OperatableStateMachine.add('Check_Hand_Space',
										LogState(text="Make sure the hands have enough space to the door", severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_To_Stand'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
