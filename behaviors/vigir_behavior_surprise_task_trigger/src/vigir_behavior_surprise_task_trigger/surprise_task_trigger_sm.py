#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_surprise_task_trigger')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_behavior_grasp_object.grasp_object_sm import GraspObjectSM
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.log_state import LogState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.footstep_plan_relative_state import FootstepPlanRelativeState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 01 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class SurpriseTaskTriggerSM(Behavior):
	'''
	Simple generic behavior for a surprise task related to triggering anything.
	'''


	def __init__(self):
		super(SurpriseTaskTriggerSM, self).__init__()
		self.name = 'Surprise Task Trigger'

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
		pull_affordance = "pull"
		affordance_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == "left" else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		# x:383 y:840, x:483 y:490
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.step_back_distance = 1.0 # meters
		_state_machine.userdata.grasp_preference = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:133 y:340, x:383 y:140
		_sm_perform_step_back_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['step_back_distance'])

		with _sm_perform_step_back_0:
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


		# x:133 y:340, x:333 y:90
		_sm_release_trigger_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'none'])

		with _sm_release_trigger_1:
			# x:82 y:78
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'Take_Hand_Back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:96 y:178
			OperatableStateMachine.add('Take_Hand_Back',
										LogState(text="Take hand slightly back", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})


		# x:133 y:540, x:433 y:240
		_sm_pull_trigger_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_pull_trigger_2:
			# x:73 y:78
			OperatableStateMachine.add('Get_Pull_Affordance',
										GetTemplateAffordanceState(identifier=pull_affordance),
										transitions={'done': 'Plan_Pull', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:92 y:228
			OperatableStateMachine.add('Plan_Pull',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Pull', 'incomplete': 'Execute_Pull', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:76 y:378
			OperatableStateMachine.add('Execute_Pull',
										ExecuteTrajectoryMsgState(controller=affordance_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})



		with _state_machine:
			# x:73 y:78
			OperatableStateMachine.add('Request_Trigger_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place trigger template"),
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
										transitions={'finished': 'Pull_Trigger', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:844 y:172
			OperatableStateMachine.add('Pull_Trigger',
										_sm_pull_trigger_2,
										transitions={'finished': 'Release_Trigger', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:836 y:272
			OperatableStateMachine.add('Release_Trigger',
										_sm_release_trigger_1,
										transitions={'finished': 'Warn_Stand', 'failed': 'Warn_Stand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'none': 'none'})

			# x:826 y:528
			OperatableStateMachine.add('Go_To_Stand_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Set_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:566 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Grasp Object', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:858 y:428
			OperatableStateMachine.add('Warn_Stand',
										LogState(text="Will go to stand now", severity=Logger.REPORT_INFO),
										transitions={'done': 'Go_To_Stand_Pose'},
										autonomy={'done': Autonomy.High})

			# x:816 y:628
			OperatableStateMachine.add('Set_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:587 y:678
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Step back?", suggestion="walk"),
										transitions={'walk': 'Perform_Step_Back', 'stand': 'finished'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:327 y:672
			OperatableStateMachine.add('Perform_Step_Back',
										_sm_perform_step_back_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'step_back_distance': 'step_back_distance'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
