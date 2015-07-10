'''Docstring'''

class AtlasDefinitions(object):
	"""docstring"""

	stand_posture = {'left_arm':  [-0.30, -1.31, +1.85, +0.49, 0.00, 0.00, 0.00],
					 'right_arm': [+0.30, +1.31, +1.85, -0.49, 0.00, 0.00, 0.00]
	}

	# Define upper and lower joint limits for both arms (order of joints same as in _joint_names attribute)
	#TODO: Get these from the URDF at runtime
	_left_upper  = [+0.785398, +1.5708, +3.14159, +2.35619, +3.011, +1.7628, +2.9671]
	_left_lower  = [-1.5708, -1.5708, 0.0, 0.0, -3.011, -1.7628, -2.9671]
	_right_upper = [+1.5708, +1.5708, +3.14159, 0.0, +3.011, +1.7628, +2.9671]
	_right_lower = [-0.785398, -1.5708, 0.0, -2.35619, -3.011, -1.7628, -2.9671]
	
	arm_joint_limits = {'left_arm':  {'upper': _left_upper,  'lower': _left_lower},
						'right_arm': {'upper': _right_upper, 'lower': _right_lower}
	}
	
	arm_joint_names = {'left_arm':  ["l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx", "l_arm_wry2"],
					   'right_arm': ["r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx", "r_arm_wry2"],
					   'left_leg':  ["l_leg_hpz", "l_leg_hpx", "l_leg_hpy", "l_leg_kny", "l_leg_aky", "l_leg_akx"], 
					   'right_leg': ["r_leg_hpz", "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", "r_leg_akx"],
					   'torso': 	["back_bkz", "back_bky", "back_bkx"],
					   'neck':      ["neck_ry"]
	}
		

	def __init__(self):

		print('The AtlasDefinitions class is not meant to be instantiated!')
