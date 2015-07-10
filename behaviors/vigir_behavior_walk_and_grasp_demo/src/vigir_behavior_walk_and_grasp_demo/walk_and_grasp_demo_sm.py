#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_walk_and_grasp_demo')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_atlas_states.tilt_head_state import TiltHeadState
from flexbe_states.input_state import InputState
from flexbe_states.log_state import LogState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_atlas_states.footstep_plan_realign_center_state import FootstepPlanRealignCenterState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from vigir_behavior_manipulation_config.manipulation_config_sm import ManipulationConfigSM
from vigir_behavior_pickup_object.pickup_object_sm import PickupObjectSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 20 2015
@author: Philipp Schillinger
'''
class WalkandGraspDemoSM(Behavior):
	'''
	Demo behavior for the scenario: Walk to the table and grab the power drill.
	'''


	def __init__(self):
		super(WalkandGraspDemoSM, self).__init__()
		self.name = 'Walk and Grasp Demo'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk to Template')
		self.add_behavior(ManipulationConfigSM, 'Manipulation Config')
		self.add_behavior(PickupObjectSM, 'Pickup Object')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 144 150 /Use_The_Drill/Lift_Drill
		# Should be able to remove this part soon.



	def create(self):
		# x:787 y:587, x:280 y:361
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.step_back_distance = 0.5 # m

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:686 y:240, x:384 y:196
		_sm_perform_walking_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'step_back_distance'])

		with _sm_perform_walking_0:
			# x:64 y:78
			OperatableStateMachine.add('Plan_Realign_Feet',
										FootstepPlanRealignCenterState(),
										transitions={'planned': 'Execute_Realign_Feet', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:624 y:378
			OperatableStateMachine.add('Perform_Step_Back',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:328 y:378
			OperatableStateMachine.add('Plan_Step_Back',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Perform_Step_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'distance': 'step_back_distance', 'plan_header': 'plan_header'})

			# x:74 y:190
			OperatableStateMachine.add('Execute_Realign_Feet',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Wait_For_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:66 y:316
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=True),
										transitions={'correct': 'Plan_Step_Back', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Full},
										remapping={'control_mode': 'control_mode'})


		# x:733 y:290, x:133 y:290
		_sm_back_to_stand_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none'])

		with _sm_back_to_stand_1:
			# x:66 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Stand_Posture', 'failed': 'failed'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.Low})

			# x:376 y:78
			OperatableStateMachine.add('Stand_Posture',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Set_Stand', 'failed': 'Set_Stand'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'side': 'none'})

			# x:666 y:78
			OperatableStateMachine.add('Set_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'finished'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})


		# x:120 y:404, x:298 y:222
		_sm_step_back_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'none', 'template_id', 'grasp_preference', 'step_back_distance'])

		with _sm_step_back_2:
			# x:76 y:28
			OperatableStateMachine.add('Go_To_Stand_Posture',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Set_To_Stand_Manipulate', 'failed': 'Set_To_Stand_Manipulate'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'side': 'none'})

			# x:66 y:140
			OperatableStateMachine.add('Set_To_Stand_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND_MANIPULATE),
										transitions={'changed': 'Perform_Walking', 'failed': 'failed'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.Low})

			# x:84 y:272
			OperatableStateMachine.add('Perform_Walking',
										_sm_perform_walking_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'step_back_distance': 'step_back_distance'})


		# x:755 y:48, x:401 y:428
		_sm_preparation_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none'], output_keys=['template_id'])

		with _sm_preparation_3:
			# x:66 y:164
			OperatableStateMachine.add('Head_Look_Straight',
										TiltHeadState(desired_tilt=TiltHeadState.STRAIGHT),
										transitions={'done': 'Place_Template', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})

			# x:566 y:163
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the placed template."),
										transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.High, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:287 y:204
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text="Have no connection to OCS!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'Decide_Input'},
										autonomy={'done': Autonomy.Off})

			# x:344 y:144
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Received wrong data format!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'Decide_Input'},
										autonomy={'done': Autonomy.Off})

			# x:483 y:44
			OperatableStateMachine.add('Fake_Input',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'template_id'})

			# x:236 y:40
			OperatableStateMachine.add('Decide_Input',
										OperatorDecisionState(outcomes=['fake_id', 'ocs_request'], hint="How do you want to provide the template?", suggestion=None),
										transitions={'fake_id': 'Fake_Input', 'ocs_request': 'Request_Template_ID'},
										autonomy={'fake_id': Autonomy.Full, 'ocs_request': Autonomy.Full})

			# x:78 y:40
			OperatableStateMachine.add('Place_Template',
										LogState(text="Please place the drill template.", severity=Logger.REPORT_HINT),
										transitions={'done': 'Decide_Input'},
										autonomy={'done': Autonomy.Full})



		with _state_machine:
			# x:44 y:72
			OperatableStateMachine.add('Preparation',
										_sm_preparation_3,
										transitions={'finished': 'Ask_Perform_Walking', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'template_id': 'template_id'})

			# x:547 y:571
			OperatableStateMachine.add('Step_Back',
										_sm_step_back_2,
										transitions={'finished': 'finished', 'failed': 'Back_To_Stand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'none': 'none', 'template_id': 'template_id', 'grasp_preference': 'grasp_preference', 'step_back_distance': 'step_back_distance'})

			# x:284 y:78
			OperatableStateMachine.add('Ask_Perform_Walking',
										OperatorDecisionState(outcomes=['walk', 'stand'], hint="Does the robot need to walk to the table?", suggestion='walk'),
										transitions={'walk': 'Walk to Template', 'stand': 'Manipulation Config'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:531 y:22
			OperatableStateMachine.add('Walk to Template',
										self.use_behavior(WalktoTemplateSM, 'Walk to Template'),
										transitions={'finished': 'Manipulation Config', 'failed': 'Walk_Manually', 'aborted': 'Walk_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:525 y:143
			OperatableStateMachine.add('Manipulation Config',
										self.use_behavior(ManipulationConfigSM, 'Manipulation Config'),
										transitions={'finished': 'Head_Look_Down', 'failed': 'Manipulation_Config_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:264 y:228
			OperatableStateMachine.add('Manipulation_Config_Manually',
										OperatorDecisionState(outcomes=["done", "abort"], hint="Make sure the robot is ready to grasp", suggestion=None),
										transitions={'done': 'Pickup Object', 'abort': 'failed'},
										autonomy={'done': Autonomy.Full, 'abort': Autonomy.Full})

			# x:769 y:78
			OperatableStateMachine.add('Walk_Manually',
										LogState(text="Guide the robot to the template manually.", severity=Logger.REPORT_HINT),
										transitions={'done': 'Manipulation Config'},
										autonomy={'done': Autonomy.Full})

			# x:245 y:485
			OperatableStateMachine.add('Back_To_Stand',
										_sm_back_to_stand_1,
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none'})

			# x:538 y:353
			OperatableStateMachine.add('Pickup Object',
										self.use_behavior(PickupObjectSM, 'Pickup Object'),
										transitions={'finished': 'Head_Look_Straight', 'failed': 'Back_To_Stand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:545 y:257
			OperatableStateMachine.add('Head_Look_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_45),
										transitions={'done': 'Pickup Object', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})

			# x:540 y:473
			OperatableStateMachine.add('Head_Look_Straight',
										TiltHeadState(desired_tilt=TiltHeadState.STRAIGHT),
										transitions={'done': 'Step_Back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
