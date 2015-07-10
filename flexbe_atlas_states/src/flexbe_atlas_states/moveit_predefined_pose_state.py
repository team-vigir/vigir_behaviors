#!/usr/bin/env python

import rospy
import actionlib

from flexbe_core import EventState, Logger
from flexbe_atlas_states.proxy import ProxyMoveitClient

"""
Created on 04/13/2014

@author: Philipp Schillinger
"""

class MoveitPredefinedPoseState(EventState):
	"""
	Uses moveit to go to one of the pre-defined poses.

	-- target_pose 			int 		Identifier of the pre-defined pose to be used.
	-- vel_scaling  		float 		Scales the velocity of the motion.
										Lower values for slower trajectories.
	-- ignore_collisions  	boolean 	Should collisions be ignored? Only pass True if you are sure that it is safe.
	-- link_paddings 		dict 		link_name (str) : padding (float) pairs
	-- is_cartesian 		boolean 	Execute as cartesian motion

	># side 				string 		Arm side, turning direction, etc.
										Possible values: {left, right, same}
 
	<= done 							Successfully executed the motion.
	<= failed 							Failed to execute the motion.

	"""

	# Arms
	STAND_POSE = 0
	SINGLE_ARM_STAND = 1 # for preventing unconstrained motion of the other
	BOTH_ARMS_SIDES = 2  # for safely holding the tool at the robot's side
	SINGLE_ARM_SIDE = 3
	STAND_POSE_UP = 4
	CALIBRATE_ARMS = 10  # for checking arm calibration with the head camera 
	WALK_POSE = 19

	# Torso
	TURN_TORSO_CENTER_POSE = 20
	TURN_TORSO_SLIGHTLY = 21
	TURN_TORSO_MORE = 22
	TURN_TORSO_FULL = 23

	# Head
	HEAD_CENTER = 30
	HEAD_SIDE = 31

	# Task-specific poses
	# Second digit is task number. Comment is pose in the Position widget.
	CAR_ENTRY_ARMS_POSE = 111 	# CarEntry
	CAR_ENTRY_LEGS_POSE = 112 	# CarEntry
	CAR_ENTRY_FORE_POSE = 113 	# CarEntryFore
	CAR_PREDRIVE_LARM_POSE = 114 	# pre_drive
	CAR_DRIVE_LARM_POSE = 115 		# drive
	CAR_DRIVE_CAMERA_POSE = 116 	# CarCamera
	DOOR_READY_POSE = 131
	DOOR_OPEN_POSE_TURNED = 132
	DOOR_OPEN_POSE_STRAIGHT = 133
	DOOR_OPEN_POSE_SIDE = 134 # push_door_3
	DOOR_PUSH_SIDE = 136
	DOOR_OPEN_TURN_MANIPULATE = 138
	DOOR_OPEN_TURN_LIDAR = 139


	POKE_READY_POSE = 151
	PREP_CUT_WALL_1 = 155
	PREP_CUT_WALL_2 = 156
	PREP_CUT_WALL_3 = 157

	LADDER_READY_POSE = 181

	
	def __init__(self, target_pose,
					   vel_scaling = 0.1,
					   ignore_collisions = False,
					   link_paddings = {},
					   is_cartesian = False):
		"""Constructor"""
		super(MoveitPredefinedPoseState, self).__init__(outcomes=['done', 'failed'],
														input_keys=['side'])

		if not rospy.has_param("behavior/robot_namespace"):
			Logger.logerr("Need to specify parameter behavior/robot_namespace at the parameter server")
			return
		
		self._robot = rospy.get_param("behavior/robot_namespace")

		self._poses = dict()
		self._poses['flor'] = dict()
		# Position mode widget: src/vigir_ocs_eui/vigir_rqt/vigir_rqt_position_mode/launch
		self._poses['flor']['left'] =  {
				 # Joint names: l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2
			1:   {'group': 'l_arm_group',	 'joints': [-0.25, -1.34, 1.88, 0.49,  0.0,   0.0, 0.0]},
			3: 	 {'group': 'l_arm_group', 	 'joints': [+0.72, -0.95, 2.7,  0.95,  0.0,  -0.4, -0.50]},
			10:  {'group': 'l_arm_group', 	 'joints': [-1.0,   0.28,  1.2,   1.6,   0.3,    0.5  ,   0.0]},
			21:  {'group': 'torso_group', 	 'joints': [+0.20, 0.00, 0.00]},
			22:  {'group': 'torso_group', 	 'joints': [+0.40, 0.00, 0.00]},
			23:  {'group': 'torso_group', 	 'joints': [+0.55, 0.00, 0.00]},
			112: {'group': 'l_leg_group', 	 'joints': [+0.00, +0.00, -1.60, +1.40, -0.50, 0.00]}, # Safety pose
			114: {'group': 'l_arm_group', 	 'joints': [+0.76, -0.94, 0.80, 2.00, +1.00, -0.20, -1.35]}, # pre_drive
			115: {'group': 'l_arm_group', 	 'joints': [+0.11, -0.16, 1.75, 1.60, +1.00, -0.90, -1.00]}, # drive
			116: {'group': 'l_arm_group', 	 'joints': []}, # We use the right hand for the camera!
			131: {'group': 'l_arm_group', 	 'joints': [-0.29, -0.22, 1.87, 2.17, -0.17, 0.81, 0.12]},
			132: {'group': 'l_arm_group', 	 'joints': [-0.70, -0.09, 1.93, 0.66, -0.15, 1.52, 0.12]},
			133: {'group': 'l_arm_group', 	 'joints': [-1.38, -0.16, 1.82, 0.57, -0.19, 1.52, 0.12]},
			134: {'group': 'l_arm_group', 	 'joints': []}, # Most probably will never be used
			151: {'group': 'l_arm_group', 	 'joints': [-1.01, -0.43, +1.32, +1.67, -0.91, +1.46, +0.98]},
			155: {'group': 'l_arm_group', 	 'joints': [0.0,  -0.37,  2.65,  1.4,  -0.2,    0.9  ,  -1.54]},
			156: {'group': 'l_arm_group', 	 'joints': [-0.32, -0.9,   2.2,   1.3,   0.5,    1.0  ,  -1.8]},
			157: {'group': 'l_arm_group', 	 'joints': [-0.45, -1.0,   2.1,   1.3,   0.5,    0.8  ,  -0.8]}
		}
		self._poses['flor']['same'] =  {
			0:   {'group': 'both_arms_group', 'joints': [-0.25, -1.34, +1.88, +0.49, +0.00, +0.00, +0.00, \
														 +0.25, +1.34, +1.88, -0.49, +0.00, +0.00, +0.00]},
			2:   {'group': 'both_arms_group', 'joints': [+0.72, -0.95,  2.7,   0.95,  0.0,   -0.4  ,  -0.5, \
														 -0.72,  0.95,  2.7,  -0.95,  0.0,    0.4  ,  -0.5]},
			19:  {'group': 'both_arms_group', 'joints': [-1.5,  -1.5,  +0.30, +0.50,  +0.0,  +0.8,  +0.00, \
														 +1.5,  +1.5,  +0.30, -0.50,  +0.0,  -0.8,  +0.00]},
			20:  {'group': 'torso_group', 	  'joints': [+0.00, 0.00, 0.00]},
			111: {'group': 'both_arms_group', 'joints': [+0.20, -1.50, +0.00, +1.72, 0.00, +0.00, 0.0, \
														 +0.00, +1.50, +0.00, -1.72, 0.00, +0.00, 0.0]},
			113: {'group': 'both_arms_group', 'joints': [+0.20, -1.50, +0.00, +1.72, 0.00, -1.57, 0.0, \
														 +0.00, +1.50, +0.00, -1.72, 0.00, +1.57, 0.0]},
			181: {'group': 'both_arms_group', 'joints': [-1.53, -0.69, +0.12, +1.47, 0.00, +0.88, 0.00, \
														 +1.53, +0.69, +0.12, -1.47, 0.00, -0.88, 0.00]}
		}
		self._poses['flor']['right'] =  {
				 # Joint names: r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx, r_arm_wry2
			1:   {'group': 'r_arm_group',	 'joints': [+0.25, 1.34, 1.88, -0.49,  0.0,   0.0,  0.0]},
			3: 	 {'group': 'r_arm_group', 	 'joints': [-0.72, 0.95, 2.7,  -0.95,  0.0,   0.4, -0.50]},
			10:  {'group': 'r_arm_group', 	 'joints': [+1.0,  -0.28,  1.2,  -1.6,   0.3,   -0.5  ,   0.0]},
			21:  {'group': 'torso_group', 	 'joints': [-0.20, 0.00, 0.00]},
			22:  {'group': 'torso_group', 	 'joints': [-0.40, 0.00, 0.00]},
			23:  {'group': 'torso_group', 	 'joints': [-0.55, 0.00, 0.00]},
			112: {'group': 'r_leg_group', 	 'joints': [+0.00, +0.00, -1.34, +1.23, 0.00, 0.00]},
			115: {'group': 'r_arm_group', 	 'joints': []}, # Driving is done with the left arm!
			116: {'group': 'r_arm_group', 	 'joints': [+0.90, 0.17, 0.50, -1.58, -0.70, +1.50, -0.34]}, # CarCamera
			131: {'group': 'r_arm_group', 	 'joints': [+0.29, 0.22, 1.87, -2.17, -0.17, -0.81, 0.12]},
			132: {'group': 'r_arm_group', 	 'joints': [+0.70, 0.09, 1.93, -0.66, -0.15, -1.52, 0.12]},
			133: {'group': 'r_arm_group', 	 'joints': [+1.38, 0.16, 1.82, -0.57, -0.19, -1.52, 0.12]},
			134: {'group': 'r_arm_group', 	 'joints': [+0.00, +0.54, +0.94, -1.04,  0.80,    0.5,   0.7]},
			151: {'group': 'r_arm_group', 	 'joints': [+1.01, +0.43, +1.32, -1.67, -0.91, -1.46, +0.98]},
			155: {'group': 'r_arm_group', 	 'joints': [+0.00, +0.37, +2.65, -1.40, -0.20, -0.90, -1.54]},
			156: {'group': 'r_arm_group', 	 'joints': [+0.32, +0.90, +2.20, -1.30, +0.50, -1.00, -1.80]},
			157: {'group': 'r_arm_group', 	 'joints': [0.45,  1.0,   2.1,  -1.3,   0.5,   -0.8  ,  -0.8]}
		}

		self._poses['thor_mang'] = dict()
		self._poses['thor_mang']['left'] = {
			1:   {'group': 'l_arm_group',	 'joints': [0.785385646194622, -0.281153767716932, 0.000600782658167331, -1.57080884130538, -0.25205140042963, 0.01563815008, 0]},
			3:   {'group': 'l_arm_group',	 'joints': [0.77, -0.27, 0.02, -1.21, -0.25, 0.02, 0]},
			4:   {'group': 'l_arm_group',	 'joints': [0.785385646194622, -0.281153767716932, 0.000600782658167331, -1.97080884130538, -0.25205140042963, 0.01563815008, 0]},
			21:  {'group': 'torso_group', 	 'joints': [+0.30, 0.03]},
			22:  {'group': 'torso_group', 	 'joints': [+0.60, 0.03]},
			23:  {'group': 'torso_group', 	 'joints': [+1.02, 0.03]},
			31:  {'group': 'head_group', 	 'joints': [1.57, 0.00]},
			131: {'group': 'l_arm_group', 	 'joints': [-1.73, -0.69, 1.75, -1.86, 0.04, -0.72, 1.63]},
			132: {'group': 'l_arm_group', 	 'joints': [-1.76, -1.13, 1.68, -0.55, 0.02, -1.81, 1.63]},
			133: {'group': 'l_arm_group', 	 'joints': [-1.70, -0.06, 1.86, -0.45, 0.08, -0.90, 1.65]},
			134: {'group': 'l_arm_group', 	 'joints': [-1.70, -0.06, 1.86, -0.45, 0.08, -0.90, 1.65]}, # placeholder
			136: {'group': 'l_arm_group', 	 'joints': [-2.31, -2.2, 1.83, -1.22, 4.1, 0.14, -2.08]},
			138: {'group': 'torso_group', 	 'joints': [0.92, 0]},
			139: {'group': 'torso_group', 	 'joints': [1.3, 0]},
			151: {'group': 'l_arm_group', 	 'joints': [-1.45, -1.26, 1.10, -2.02, -0.19, 0.04, -2.86]},
			155: {'group': 'l_arm_group',	 'joints': [-0.25, -0.75, 0.02, -1.21, -0.38, -1.36, -0.9]},
			156: {'group': 'l_arm_group',	 'joints': [-0.25, -0.25, 0.02, -1.21, -0.38, -1.36, -0.9]},
			157: {'group': 'l_arm_group',	 'joints': [-0.25, -0.13, 0.02, -1.21, -0.13, -1.55, 0.41]}
		}
		self._poses['thor_mang']['same'] = {
			0:   {'group': 'both_arms_group', 'joints': [0.79, -0.27, 0, -1.57, 1.55, 0, 0, -0.79, 0.27, 0, 1.57, -1.55, 0, 0]},
			2:   {'group': 'both_arms_group', 'joints': [-0.37, 0.0, 1.57, 0, 0, 0, 0, 0.37, 0.0, 1.57, 0, 0, 0, 0]},
			20:  {'group': 'torso_group', 	  'joints': [0.00, 0.03]},
			30:  {'group': 'head_group', 	  'joints': [0.00, 0.00]},
			181: {'group': 'both_arms_group', 'joints': [0.785385646194622, -0.281153767716932, 0.000600782658167331, -1.57080884130538, -0.25205140042963, 0.01563815008, 0, -0.785385646194622, 0.281153767716932, -0.000600782658167331, 1.57080884130538, 0.25205140042963, -0.01563815008, 0]}
		}
		self._poses['thor_mang']['right'] = {
			1:   {'group': 'r_arm_group',	 'joints': [-0.785385646194622, 0.281153767716932, -0.000600782658167331, 1.57080884130538, 0.25205140042963, -0.01563815008, 0]},
			3:   {'group': 'r_arm_group',	 'joints': [-0.77, 0.27, -0.02, 1.21, 0.25, -0.02, 0]},
			4:   {'group': 'r_arm_group',	 'joints': [-0.785385646194622, 0.281153767716932, -0.000600782658167331, 1.97080884130538, 0.25205140042963, -0.01563815008, 0]},
			21:  {'group': 'torso_group', 	 'joints': [-0.30, 0.03]},
			22:  {'group': 'torso_group', 	 'joints': [-0.60, 0.03]},
			23:  {'group': 'torso_group', 	 'joints': [-1.02, 0.03]},
			31:  {'group': 'head_group', 	 'joints': [-1.57, 0.00]},
			131: {'group': 'r_arm_group', 	 'joints': [1.73, 0.69, -1.75, 1.86, -0.04, 0.72, -1.63]},
			132: {'group': 'r_arm_group', 	 'joints': [1.76, 1.13, -1.68, 0.55, -0.02, 1.81, -1.63]},
			133: {'group': 'r_arm_group', 	 'joints': [1.70, 0.06, -1.86, 0.45, -0.08, 0.90, -1.65]},
			134: {'group': 'r_arm_group', 	 'joints': [1.70, 0.06, -1.86, 0.45, -0.08, 0.90, -1.65]}, # placeholder
			136: {'group': 'r_arm_group', 	 'joints': [2.31, 2.2, -1.83, 1.22, -4.1, -0.14, 2.08]},
			138: {'group': 'torso_group', 	 'joints': [-0.92, 0]},
			139: {'group': 'torso_group', 	 'joints': [-1.3, 0]},
			151: {'group': 'r_arm_group', 	 'joints': [1.45, 1.26, -1.10, 2.02, 0.19, -0.04, 2.86]},
			155: {'group': 'r_arm_group',	 'joints': [0.25, 0.75, -0.02, 1.21, 0.38, 1.36, 0.9]},
			156: {'group': 'r_arm_group',	 'joints': [0.25, 0.25, -0.02, 1.21, 0.38, 1.36, 0.9]},
			157: {'group': 'r_arm_group',	 'joints': [0.25, 0.13, -0.02, 1.21, 0.13, 1.55, -0.41]}
		}
		
		self._target_pose = target_pose
		self._vel_scaling = vel_scaling
		self._ignore_collisions = ignore_collisions
		self._link_paddings = link_paddings
		self._is_cartesian = is_cartesian

		self._client = ProxyMoveitClient()
		
		self._failed = False


	def execute(self, userdata):
		"""Execute this state"""
		if self._failed:
			return 'failed'

		if self._client.finished():
			if self._client.success():
				return 'done'
			else:
				Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
				self._failed = True
				return 'failed'
	
	
	def on_enter(self, userdata):
		self._failed = False

		planning_group = ""
		joint_values = []
		if userdata.side == 'left' and self._target_pose in self._poses[self._robot]['left'].keys():
			planning_group = self._poses[self._robot]['left'][self._target_pose]['group']
			joint_values = self._poses[self._robot]['left'][self._target_pose]['joints']
		elif userdata.side == 'right' and self._target_pose in self._poses[self._robot]['right'].keys():
			planning_group = self._poses[self._robot]['right'][self._target_pose]['group']
			joint_values = self._poses[self._robot]['right'][self._target_pose]['joints']
		elif self._target_pose in self._poses[self._robot]['same'].keys():
			planning_group = self._poses[self._robot]['same'][self._target_pose]['group']
			joint_values = self._poses[self._robot]['same'][self._target_pose]['joints']
		else:
			Logger.logwarn('Specified target pose %d is not specified for %s side' % (self._target_pose, userdata.side))
			self._failed = True
			return

		# create the motion goal
		self._client.new_goal(planning_group)
		self._client.add_joint_values(joint_values)
		self._client.set_velocity_scaling(self._vel_scaling)
		self._client.set_collision_avoidance(self._ignore_collisions)
		self._client.add_link_padding(link_paddings = self._link_paddings) #dict
		if self._is_cartesian:
			self._client.set_cartesian_motion()

		# execute the motion
		try: 
			self._client.start_execution()
		except Exception as e:
			Logger.logwarn('Was unable to execute move group request:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.finished():
			self._client.cancel()
			Logger.loginfo("Cancelled active action goal.")
