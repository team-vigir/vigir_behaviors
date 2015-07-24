import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

'''Docstring'''

class RobotFunctions(object):
	'''docstring'''

	def __init__(self):

		print('The RobotFunctions class is not meant to be instantiated!')

	@staticmethod
	def gen_trajectory_from_joint_configuration(sides, joints, joint_config, time = 5.0):
		
		resulting_trajectory = dict()

		for arm in sides:

			trajectory = JointTrajectory()
			trajectory.joint_names = joints[arm]

			point = JointTrajectoryPoint()
			point.positions = joint_config[arm]
			point.time_from_start = rospy.Duration.from_sec(time)
			trajectory.points.append(point)

			resulting_trajectory[arm] = trajectory

		return resulting_trajectory
