#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_walk_to_template')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.log_state import LogState
from flexbe_states.input_state import InputState
from vigir_flexbe_states.check_current_control_mode_state import CheckCurrentControlModeState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.create_step_goal_state import CreateStepGoalState
from vigir_flexbe_states.plan_footsteps_state import PlanFootstepsState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from vigir_flexbe_states.get_template_stand_pose_state import GetTemplateStandPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Apr 04 2015
@author: Philipp Schillinger
'''
class WalktoTemplateSM(Behavior):
	'''
	Lets the robot walk to the specified template's stand pose.
	'''


	def __init__(self):
		super(WalktoTemplateSM, self).__init__()
		self.name = 'Walk to Template'

		# parameters of this behavior
		self.add_parameter('parameter_set', 'drc_step_2D')
		self.add_parameter('hand_side', 'left')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1105 y:470, x:375 y:272, x:315 y:498
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['grasp_preference', 'hand_side', 'template_id'])
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.template_id = None # provide or request

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:747 y:489, x:337 y:62, x:582 y:379
		_sm_planning_pipeline_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['stand_pose'], output_keys=['plan_header'])

		with _sm_planning_pipeline_0:
			# x:34 y:57
			OperatableStateMachine.add('Create_Step_Goal',
										CreateStepGoalState(pose_is_pelvis=True),
										transitions={'done': 'Plan_To_Waypoint', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'target_pose': 'stand_pose', 'step_goal': 'step_goal'})

			# x:553 y:481
			OperatableStateMachine.add('Modify_Plan',
										InputState(request=InputState.FOOTSTEP_PLAN_HEADER, message='Modify plan, VALIDATE, and confirm.'),
										transitions={'received': 'finished', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Low, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'plan_header'})

			# x:34 y:484
			OperatableStateMachine.add('Plan_To_Waypoint',
										PlanFootstepsState(mode=self.parameter_set),
										transitions={'planned': 'Modify_Plan', 'failed': 'Decide_Replan_without_Collision'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})

			# x:139 y:314
			OperatableStateMachine.add('Decide_Replan_without_Collision',
										OperatorDecisionState(outcomes=['replan', 'fail'], hint='Try replanning without collision avoidance.', suggestion='replan'),
										transitions={'replan': 'Replan_without_Collision', 'fail': 'failed'},
										autonomy={'replan': Autonomy.Low, 'fail': Autonomy.High})

			# x:319 y:406
			OperatableStateMachine.add('Replan_without_Collision',
										PlanFootstepsState(mode='drc_step_no_collision'),
										transitions={'planned': 'Modify_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})



		with _state_machine:
			# x:265 y:28
			OperatableStateMachine.add('Decide_Request_Template',
										DecisionState(outcomes=['request', 'continue'], conditions=lambda x: 'continue' if x is not None else 'request'),
										transitions={'request': 'Request_Template', 'continue': 'Get_Stand_Pose'},
										autonomy={'request': Autonomy.Low, 'continue': Autonomy.Off},
										remapping={'input_value': 'template_id'})

			# x:1033 y:106
			OperatableStateMachine.add('Increment_Stand_Pose',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Inform_About_Retry'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:1162 y:29
			OperatableStateMachine.add('Inform_About_Retry',
										LogState(text="Stand pose choice failed. Trying again.", severity=Logger.REPORT_INFO),
										transitions={'done': 'Get_Stand_Pose'},
										autonomy={'done': Autonomy.Off})

			# x:567 y:118
			OperatableStateMachine.add('Inform_About_Fail',
										LogState(text="Unable to find a suitable stand pose for the template.", severity=Logger.REPORT_WARN),
										transitions={'done': 'Decide_Repeat_Request'},
										autonomy={'done': Autonomy.Off})

			# x:554 y:274
			OperatableStateMachine.add('Get_Goal_from_Operator',
										InputState(request=InputState.WAYPOINT_GOAL_POSE, message="Provide a waypoint in front of the template."),
										transitions={'received': 'Walk_To_Waypoint', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'plan_header'})

			# x:279 y:110
			OperatableStateMachine.add('Request_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Specify target template"),
										transitions={'received': 'Get_Stand_Pose', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:825 y:461
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=True),
										transitions={'correct': 'finished', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Full},
										remapping={'control_mode': 'control_mode'})

			# x:1143 y:277
			OperatableStateMachine.add('Decide_Stand_Preference',
										OperatorDecisionState(outcomes=["same", "next"], hint="Same or next stand pose?", suggestion="next"),
										transitions={'same': 'Inform_About_Retry', 'next': 'Increment_Stand_Pose'},
										autonomy={'same': Autonomy.Full, 'next': Autonomy.Full})

			# x:842 y:152
			OperatableStateMachine.add('Planning_Pipeline',
										_sm_planning_pipeline_0,
										transitions={'finished': 'Walk_To_Waypoint', 'failed': 'Decide_Stand_Preference', 'aborted': 'Decide_Stand_Preference'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'stand_pose': 'stand_pose', 'plan_header': 'plan_header'})

			# x:833 y:276
			OperatableStateMachine.add('Walk_To_Waypoint',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Wait_For_Stand', 'failed': 'Decide_Stand_Preference'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'plan_header': 'plan_header'})

			# x:554 y:195
			OperatableStateMachine.add('Decide_Repeat_Request',
										OperatorDecisionState(outcomes=['repeat_id', 'request_goal'], hint=None, suggestion=None),
										transitions={'repeat_id': 'Request_Template', 'request_goal': 'Get_Goal_from_Operator'},
										autonomy={'repeat_id': Autonomy.Low, 'request_goal': Autonomy.High})

			# x:547 y:27
			OperatableStateMachine.add('Get_Stand_Pose',
										GetTemplateStandPoseState(),
										transitions={'done': 'Planning_Pipeline', 'failed': 'Inform_About_Fail', 'not_available': 'Inform_About_Fail'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'stand_pose': 'stand_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
