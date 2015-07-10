#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_exploration_demo')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.calculation_state import CalculationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.plan_footsteps_state import PlanFootstepsState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.log_state import LogState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.send_to_operator_state import SendToOperatorState
from flexbe_atlas_states.get_camera_image_state import GetCameraImageState
from flexbe_atlas_states.get_laserscan_state import GetLaserscanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Wed Mar 04 2015
@author: Philipp Schillinger
'''
class ExplorationDemoSM(Behavior):
	'''
	Explores an area and collects camera images of it.
	'''


	def __init__(self):
		super(ExplorationDemoSM, self).__init__()
		self.name = 'Exploration Demo'

		# parameters of this behavior
		self.add_parameter('waypoint_distance', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]


	def create(self):
		waypoint_distance = 1
		# x:92 y:394, x:775 y:176
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.none = None
		_state_machine.userdata.waypoint = PoseStamped()		

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:105 y:518, x:331 y:110, x:346 y:384
		_sm_process_waypoint_0 = OperatableStateMachine(outcomes=['finished', 'waypoint_failure', 'data_failure'], input_keys=['waypoint'])

		with _sm_process_waypoint_0:
			# x:58 y:44
			OperatableStateMachine.add('Plan_To_Waypoint',
										PlanFootstepsState(mode=PlanFootstepsState.MODE_STEP_2D, pose_is_pelvis=False),
										transitions={'planned': 'Perform_Walking', 'failed': 'waypoint_failure'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose': 'waypoint', 'footstep_plan': 'footstep_plan'})

			# x:40 y:154
			OperatableStateMachine.add('Perform_Walking',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Take_Camera_Image', 'failed': 'waypoint_failure'},
										autonomy={'finished': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'footstep_plan': 'footstep_plan'})

			# x:42 y:376
			OperatableStateMachine.add('Send_Image_To_Operator',
										SendToOperatorState(),
										transitions={'done': 'finished', 'no_connection': 'data_failure'},
										autonomy={'done': Autonomy.Low, 'no_connection': Autonomy.Low},
										remapping={'data': 'camera_img'})

			# x:53 y:266
			OperatableStateMachine.add('Take_Camera_Image',
										GetCameraImageState(),
										transitions={'done': 'Send_Image_To_Operator'},
										autonomy={'done': Autonomy.Off},
										remapping={'camera_img': 'camera_img'})

			# x:190 y:266
			OperatableStateMachine.add('Take_Laser_Scan',
										GetLaserscanState(),
										transitions={'done': 'Send_Image_To_Operator'},
										autonomy={'done': Autonomy.Off},
										remapping={'laserscan': 'laserscan'})


		# x:30 y:365
		_sm_generate_waypoint_1 = OperatableStateMachine(outcomes=['finished'], input_keys=['waypoint', 'none'], output_keys=['waypoint_out'])

		with _sm_generate_waypoint_1:
			# x:32 y:50
			OperatableStateMachine.add('Init_Radius',
										CalculationState(calculation=lambda x: 0.2),
										transitions={'done': 'Calculate_Next_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'sdv'})

			# x:173 y:129
			OperatableStateMachine.add('Calculate_Next_Pose',
										FlexibleCalculationState(calculation=self.calc_pose, input_keys=["pose", "radius"]),
										transitions={'done': 'Determine_Waypoint_Valid'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose': 'waypoint', 'radius': 'sdv', 'output_value': 'waypoint_out'})

			# x:162 y:260
			OperatableStateMachine.add('Determine_Waypoint_Valid',
										PlanFootstepsState(mode=PlanFootstepsState.MODE_STEP_2D, pose_is_pelvis=False),
										transitions={'planned': 'finished', 'failed': 'Increase_Radius'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose': 'waypoint_out', 'footstep_plan': 'footstep_plan'})

			# x:299 y:193
			OperatableStateMachine.add('Increase_Radius',
										CalculationState(calculation=lambda s: s + 0.1),
										transitions={'done': 'Calculate_Next_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'sdv', 'output_value': 'sdv'})



		with _state_machine:
			# x:33 y:45
			OperatableStateMachine.add('Generate_Waypoint',
										_sm_generate_waypoint_1,
										transitions={'finished': 'Process_Waypoint'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint', 'none': 'none', 'waypoint_out': 'waypoint'})

			# x:726 y:51
			OperatableStateMachine.add('Abort_After_Failure',
										OperatorDecisionState(outcomes=["continue", "abort"], hint="Continue with next position regardless of failure?", suggestion="continue"),
										transitions={'continue': 'Generate_Waypoint', 'abort': 'failed'},
										autonomy={'continue': Autonomy.Full, 'abort': Autonomy.Full})

			# x:513 y:282
			OperatableStateMachine.add('Log_Data_Failure',
										LogState(text="Failed to send captured data!", severity=Logger.REPORT_WARN),
										transitions={'done': 'Abort_After_Failure'},
										autonomy={'done': Autonomy.Off})

			# x:253 y:178
			OperatableStateMachine.add('Process_Waypoint',
										_sm_process_waypoint_0,
										transitions={'finished': 'Check_Finished', 'waypoint_failure': 'Log_Waypoint_Failure', 'data_failure': 'Log_Data_Failure'},
										autonomy={'finished': Autonomy.Inherit, 'waypoint_failure': Autonomy.Inherit, 'data_failure': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint'})

			# x:42 y:284
			OperatableStateMachine.add('Check_Finished',
										OperatorDecisionState(outcomes=["finished", "next"], hint="Going to next position?", suggestion="next"),
										transitions={'finished': 'finished', 'next': 'Generate_Waypoint'},
										autonomy={'finished': Autonomy.Full, 'next': Autonomy.High})

			# x:505 y:120
			OperatableStateMachine.add('Log_Waypoint_Failure',
										LogState(text="Failed to walk to next waypoint!", severity=Logger.REPORT_WARN),
										transitions={'done': 'Abort_After_Failure'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def calc_pose(self, input_list):
		pose = input_list[0]
		sdv = input_list[1]

		out_pose = PoseStamped()
		out_pose.pose.position.x = pose.pose.position.x + random.gauss(self.waypoint_distance, sdv)
		out_pose.pose.position.y = pose.pose.position.y + random.gauss(0, sdv)
		out_pose.pose.orientation = Quaternion(w=1)
		out_pose.header = Header(frame_id="world")

		print out_pose

		Logger.loginfo("Calculated new positon (%s/%s)." % (str(out_pose.pose.position.x), str(out_pose.pose.position.y)));

		return out_pose


	# [/MANUAL_FUNC]
