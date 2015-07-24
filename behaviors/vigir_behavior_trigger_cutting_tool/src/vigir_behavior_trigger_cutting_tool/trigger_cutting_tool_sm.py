#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_trigger_cutting_tool')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.calculation_state import CalculationState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.get_wrist_pose_state import GetWristPoseState
from vigir_flexbe_states.get_pose_in_frame_state import GetPoseInFrameState
from flexbe_states.log_state import LogState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
import math
import copy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from flexbe_core.proxy import ProxyPublisher
# [/MANUAL_IMPORT]


'''
Created on Tue May 12 2015
@author: Dorothea Koert, Philipp Schillinger
'''
class TriggerCuttingToolSM(Behavior):
	'''
	Switch the cutting tool on or off.
	'''


	def __init__(self):
		super(TriggerCuttingToolSM, self).__init__()
		self.name = 'Trigger Cutting Tool'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		self._scaling_factor = 0

		self._pub1 = rospy.Publisher('/bla1', PoseStamped)
		self._pub2 = rospy.Publisher('/bla2', PoseStamped)

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 453 228 
		# Start spiral again with its origin at the current position

		# ! 3 256 
		# Skip predefined pre_poke_pose for now



	def create(self):
		number_of_points = 9
		scaling_factor = 2 # for spiral pattern
		attempts_per_point = 3
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		poking_stick_frame = self.hand_side + '_poking_stick'
		# x:183 y:40, x:283 y:290
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.none = None
		_state_machine.userdata.hand_side = self.hand_side

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		self._scaling_factor = scaling_factor

		# [/MANUAL_CREATE]

		# x:637 y:484, x:391 y:152
		_sm_calculate_poke_poses_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'], output_keys=['poke_waypoints', 'poke_frame_id', 'pre_poke_waypoints', 'pre_poke_frame_id'])

		with _sm_calculate_poke_poses_0:
			# x:63 y:78
			OperatableStateMachine.add('Get_Current_Endeffector_Pose',
										GetWristPoseState(),
										transitions={'done': 'Transform_Endeffector_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'hand_side': 'hand_side', 'wrist_pose': 'wrist_pose'})

			# x:67 y:178
			OperatableStateMachine.add('Transform_Endeffector_Pose',
										GetPoseInFrameState(target_frame=poking_stick_frame),
										transitions={'done': 'Translate_To_Poke_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose_in': 'wrist_pose', 'pose_out': 'pre_poke_pose'})

			# x:77 y:278
			OperatableStateMachine.add('Translate_To_Poke_Pose',
										CalculationState(calculation=self.calc_poke_pose),
										transitions={'done': 'Pre_Poke_Pose_To_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pre_poke_pose', 'output_value': 'poke_pose'})

			# x:324 y:378
			OperatableStateMachine.add('Poke_Pose_To_Waypoints',
										CalculationState(calculation=lambda x: [x.pose]),
										transitions={'done': 'Set_Poke_Waypoints_Frame'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'poke_pose', 'output_value': 'poke_waypoints'})

			# x:569 y:378
			OperatableStateMachine.add('Set_Poke_Waypoints_Frame',
										CalculationState(calculation=lambda x: x.header.frame_id),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'poke_pose', 'output_value': 'poke_frame_id'})

			# x:313 y:278
			OperatableStateMachine.add('Pre_Poke_Pose_To_Waypoints',
										CalculationState(calculation=lambda x: [x.pose]),
										transitions={'done': 'Set_Pre_Poke_Waypoints_Frame'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pre_poke_pose', 'output_value': 'pre_poke_waypoints'})

			# x:558 y:278
			OperatableStateMachine.add('Set_Pre_Poke_Waypoints_Frame',
										CalculationState(calculation=lambda x: x.header.frame_id),
										transitions={'done': 'Poke_Pose_To_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pre_poke_pose', 'output_value': 'pre_poke_frame_id'})


		# x:433 y:40, x:441 y:253
		_sm_poke_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'hand_side', 'poke_waypoints', 'poke_frame_id', 'pre_poke_waypoints', 'pre_poke_frame_id'])

		with _sm_poke_1:
			# x:49 y:78
			OperatableStateMachine.add('Init_Inner_Index',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Plan_To_Poke_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'poking_index'})

			# x:76 y:328
			OperatableStateMachine.add('Move_To_Poke_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Plan_To_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:626 y:378
			OperatableStateMachine.add('Move_To_Pre_Poke_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Increase_Inner_Index', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:636 y:178
			OperatableStateMachine.add('Increase_Inner_Index',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'Check_Inner_Index'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'poking_index', 'output_value': 'poking_index'})

			# x:392 y:178
			OperatableStateMachine.add('Check_Inner_Index',
										DecisionState(outcomes=['continue','finished'], conditions=lambda x: 'continue' if x<attempts_per_point else 'finished'),
										transitions={'continue': 'Plan_To_Poke_Pose', 'finished': 'finished'},
										autonomy={'continue': Autonomy.Low, 'finished': Autonomy.Low},
										remapping={'input_value': 'poking_index'})

			# x:46 y:178
			OperatableStateMachine.add('Plan_To_Poke_Pose',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.2, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Poke_Pose', 'incomplete': 'Move_To_Poke_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'waypoints': 'poke_waypoints', 'hand': 'hand_side', 'frame_id': 'poke_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:296 y:428
			OperatableStateMachine.add('Plan_To_Pre_Poke_Pose',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.5, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pre_Poke_Pose', 'incomplete': 'Move_To_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'waypoints': 'pre_poke_waypoints', 'hand': 'hand_side', 'frame_id': 'pre_poke_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})



		with _state_machine:
			# x:51 y:78
			OperatableStateMachine.add('Init_Index',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Move_To_Ready_Pose'},
										autonomy={'done': Autonomy.High},
										remapping={'input_value': 'none', 'output_value': 'attempt_index'})

			# x:29 y:278
			OperatableStateMachine.add('Prepare_Pre_Poke_Pose',
										CalculationState(calculation=self.suggest_pre_poke_pose),
										transitions={'done': 'Plan_To_Pre_Poke_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'pre_poke_pose'})

			# x:28 y:378
			OperatableStateMachine.add('Plan_To_Pre_Poke_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'target_pose': 'pre_poke_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:26 y:478
			OperatableStateMachine.add('Move_To_Pre_Poke_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Adjust_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:344 y:122
			OperatableStateMachine.add('Poke',
										_sm_poke_1,
										transitions={'finished': 'Check_Cutting_Tool_Status', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'hand_side': 'hand_side', 'poke_waypoints': 'poke_waypoints', 'poke_frame_id': 'poke_frame_id', 'pre_poke_waypoints': 'pre_poke_waypoints', 'pre_poke_frame_id': 'pre_poke_frame_id'})

			# x:321 y:28
			OperatableStateMachine.add('Check_Cutting_Tool_Status',
										OperatorDecisionState(outcomes=['on', 'off', 'reset_iteration'], hint="Is the cutting tool on?", suggestion=None),
										transitions={'on': 'finished', 'off': 'Increment_Index', 'reset_iteration': 'Reset_Index'},
										autonomy={'on': Autonomy.Full, 'off': Autonomy.Full, 'reset_iteration': Autonomy.Full})

			# x:798 y:28
			OperatableStateMachine.add('Increment_Index',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'Calc_Next_Pre_Poke_Pose'},
										autonomy={'done': Autonomy.High},
										remapping={'input_value': 'attempt_index', 'output_value': 'attempt_index'})

			# x:26 y:178
			OperatableStateMachine.add('Move_To_Ready_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.POKE_READY_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Adjust_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'side': 'hand_side'})

			# x:321 y:372
			OperatableStateMachine.add('Calculate_Poke_Poses',
										_sm_calculate_poke_poses_0,
										transitions={'finished': 'Poke', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'poke_waypoints': 'poke_waypoints', 'poke_frame_id': 'poke_frame_id', 'pre_poke_waypoints': 'pre_poke_waypoints', 'pre_poke_frame_id': 'pre_poke_frame_id'})

			# x:82 y:578
			OperatableStateMachine.add('Adjust_Pre_Poke_Pose',
										LogState(text="Adjust pose so that the template is in front of the poking stick", severity=Logger.REPORT_HINT),
										transitions={'done': 'Get_Current_Endeffector_Pose'},
										autonomy={'done': Autonomy.Full})

			# x:601 y:178
			OperatableStateMachine.add('Reset_Index',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Get_Current_Endeffector_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'attempt_index', 'output_value': 'attempt_index'})

			# x:764 y:278
			OperatableStateMachine.add('Plan_To_Next_Pre_Poke_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Next_Pre_Poke_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'target_pose': 'pre_poke_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:762 y:378
			OperatableStateMachine.add('Move_To_Next_Pre_Poke_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Calculate_Poke_Poses', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:773 y:178
			OperatableStateMachine.add('Calc_Next_Pre_Poke_Pose',
										FlexibleCalculationState(calculation=self.calc_pre_poke_pose, input_keys=['index', 'init_pre_poke_pose']),
										transitions={'done': 'Plan_To_Next_Pre_Poke_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'index': 'attempt_index', 'init_pre_poke_pose': 'init_pre_poke_pose', 'output_value': 'pre_poke_pose'})

			# x:563 y:578
			OperatableStateMachine.add('Get_Current_Endeffector_Pose',
										GetWristPoseState(),
										transitions={'done': 'Transform_Endeffector_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'hand_side': 'hand_side', 'wrist_pose': 'wrist_pose'})

			# x:317 y:478
			OperatableStateMachine.add('Transform_Endeffector_Pose',
										GetPoseInFrameState(target_frame=poking_stick_frame),
										transitions={'done': 'Calculate_Poke_Poses', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose_in': 'wrist_pose', 'pose_out': 'init_pre_poke_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def suggest_pre_poke_pose(self, none):
		pre_poke_pose = PoseStamped()
		pre_poke_pose.header.stamp = rospy.Time.now()

		pre_poke_pose.header.frame_id = self.hand_side + '_poking_stick'

		if self.hand_side == 'left':
			pre_poke_pose.pose.orientation = Quaternion(0.606, -0.377, -0.498, -0.491)
			pre_poke_pose.pose.position.x = 0.048 + 0.1 # this saves the robot (for testing)
			pre_poke_pose.pose.position.y = 0.254
			pre_poke_pose.pose.position.z = 0.103
		if self.hand_side == 'right':
			pre_poke_pose.pose.orientation = Quaternion(0.606, -0.377, -0.498, -0.491) # TODO change to correct orientation
			Logger.logwarn('Orientation not correctly set for right arm!')
			pre_poke_pose.pose.position.x = 0.048 + 0.1 # this saves the robot (for testing)
			pre_poke_pose.pose.position.y = -0.254
			pre_poke_pose.pose.position.z = 0.103

		return pre_poke_pose


	def calc_pre_poke_pose(self, args):
		index = args[0]
		init_pre_poke_pose = args[1]
		
		goal_pose = PoseStamped()
		goal_pose.header.stamp = rospy.Time.now()
		goal_pose.header.frame_id = init_pre_poke_pose.header.frame_id
		goal_pose.pose.orientation = init_pre_poke_pose.pose.orientation

		spiral_idx = math.sqrt(250*idx)
		y_offset = 0.02 * spiral_idx * math.cos(0.2 * spiral_idx)
		z_offset = 0.02 * spiral_idx * math.sin(0.2 * spiral_idx)

		goal_pose.pose.position.x = init_pre_poke_pose.pose.x
		goal_pose.pose.position.y = init_pre_poke_pose.pose.y 	+ 0.01 * y_offset * self._scaling_factor
		goal_pose.pose.position.z = init_pre_poke_pose.pose.z	+ 0.01 * z_offset * self._scaling_factor

		print goal_pose

		return goal_pose


	def calc_poke_pose(self, pre_poke_pose):
				
		self._pub1.publish(pre_poke_pose)
		poke_pose = copy.deepcopy(pre_poke_pose)
		poke_pose.pose.position.x -= 0.1
		self._pub2.publish(poke_pose)
		return poke_pose





	# [/MANUAL_FUNC]
