from flexbe_behaviors.atlas_definitions import AtlasDefinitions
from flexbe_behaviors.robot_functions import RobotFunctions

'''Docstring'''

class AtlasFunctions(object):
	"""docstring"""
	
	def __init__(self):

		print('The AtlasFunctions class is not meant to be instantiated!')

	@staticmethod
	def gen_stand_posture_trajectory(time = 5.0):
		
		joint_config = AtlasDefinitions.stand_posture

		return AtlasFunctions.gen_arm_trajectory_from_joint_configuration(joint_config, time)

	@staticmethod
	def gen_arm_trajectory_from_joint_configuration(joint_config, time = 5.0):
		'''docstring'''

		sides = ['left_arm', 'right_arm']
		joints = AtlasDefinitions.arm_joint_names

		return RobotFunctions.gen_trajectory_from_joint_configuration(sides, joints, joint_config, time)
