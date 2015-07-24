#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_pickup_object')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_behavior_grasp_object.grasp_object_sm import GraspObjectSM
from vigir_flexbe_states.attach_object_state import AttachObjectState
from vigir_flexbe_states.get_wrist_pose_state import GetWristPoseState
from flexbe_states.calculation_state import CalculationState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from vigir_flexbe_states.get_pose_in_frame_state import GetPoseInFrameState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 11 2015
@author: Philipp Schillinger
'''
class PickupObjectSM(Behavior):
	'''
	Grasps and picks up any object in front of the robot.
	'''


	def __init__(self):
		super(PickupObjectSM, self).__init__()
		self.name = 'Pickup Object'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')

		# references to used behaviors
		self.add_behavior(GraspObjectSM, 'Grasp_Object')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		cart_lift = 0.20 # meters
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		wrist_frame_id = self.hand_side[0] + '_hand'
		# x:220 y:518, x:463 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.template_id = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:71 y:304, x:597 y:174
		_sm_lift_object_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id'])

		with _sm_lift_object_0:
			# x:255 y:28
			OperatableStateMachine.add('Get_Wrist_Pose',
										GetWristPoseState(),
										transitions={'done': 'Set_Target_for_Lifting', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side', 'wrist_pose': 'wrist_pose'})

			# x:525 y:29
			OperatableStateMachine.add('Set_Target_for_Lifting',
										CalculationState(calculation=self.lift_template(cart_lift, False)),
										transitions={'done': 'Transform_to_Wrist_Frame'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'wrist_pose', 'output_value': 'lift_target'})

			# x:887 y:296
			OperatableStateMachine.add('Prepare_Plan_Frame_Wrist',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Plan_Cartesian_Lift'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'lift_target_hand', 'output_value': 'hand_frame_id'})

			# x:523 y:296
			OperatableStateMachine.add('Plan_Cartesian_Lift',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=False, include_torso=False, keep_endeffector_orientation=True, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Perform_Lifting', 'incomplete': 'Perform_Lifting', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'waypoints': 'lift_waypoints', 'hand': 'hand_side', 'frame_id': 'hand_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:886 y:29
			OperatableStateMachine.add('Transform_to_Wrist_Frame',
										GetPoseInFrameState(target_frame=wrist_frame_id),
										transitions={'done': 'Convert_to_List_of_Poses', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'pose_in': 'lift_target', 'pose_out': 'lift_target_hand'})

			# x:888 y:160
			OperatableStateMachine.add('Convert_to_List_of_Poses',
										CalculationState(calculation=lambda pose: [pose.pose]),
										transitions={'done': 'Prepare_Plan_Frame_Wrist'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'lift_target_hand', 'output_value': 'lift_waypoints'})

			# x:206 y:297
			OperatableStateMachine.add('Perform_Lifting',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})



		with _state_machine:
			# x:187 y:25
			OperatableStateMachine.add('Grasp_Object',
										self.use_behavior(GraspObjectSM, 'Grasp_Object'),
										transitions={'finished': 'Confirm-Successful_Grasp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:181 y:275
			OperatableStateMachine.add('Attach_Object_to_Hand',
										AttachObjectState(),
										transitions={'done': 'Lift_Object', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'template_pose': 'template_pose'})

			# x:190 y:383
			OperatableStateMachine.add('Lift_Object',
										_sm_lift_object_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:176 y:159
			OperatableStateMachine.add('Confirm-Successful_Grasp',
										LogState(text='Confirm that the grasp was successful!', severity=Logger.REPORT_HINT),
										transitions={'done': 'Attach_Object_to_Hand'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def lift_template(self, lift_height, cartesian):
		'''Add some height to the template's pose.'''
		
		def create_waypoint_list(template_pose): #
			temp = template_pose.pose.position.z
			template_pose.pose.position.z += lift_height
			Logger.loginfo('Changed template height from %f to %f' % (temp, template_pose.pose.position.z))
			print template_pose.pose
			return [template_pose.pose] if cartesian else template_pose
		
		return create_waypoint_list

	# [/MANUAL_FUNC]
