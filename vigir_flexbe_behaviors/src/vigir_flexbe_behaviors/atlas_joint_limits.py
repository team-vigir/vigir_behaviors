#!/usr/bin/env python

import rospy
import os, sys

from urdf_parser_py.urdf import URDF
import rosparam

def main():
	
	get_atlas_joint_limits(output = True)

def get_atlas_joint_limits(output = False):
	
	joint_limits = dict()

	for side in ["left", "right"]:

		joint_limits[side] = dict()
		joint_limits[side]['lower'] = list()
		joint_limits[side]['upper'] = list()

		description = AtlasRobotDescription(side)

		for index, joint in enumerate(description.chain):

			robot_joint = description.robot.joints[description.joint_list[joint.name]]
			lower_limit = robot_joint.limit.lower
			upper_limit = robot_joint.limit.upper

			joint_limits[side]['lower'].append(lower_limit)
			joint_limits[side]['upper'].append(upper_limit)

			if output:
				print("Upper and lower limits for joint %s:" % joint.name)
				print upper_limit
				print lower_limit

	return joint_limits

class AtlasRobotDescription(object):
	
	def __init__(self, side):

		# Redirect annoying output of upcoming URDF command
		devnull = open(os.devnull, 'w')
		sys.stdout, sys.stderr = devnull, devnull
		
		self.robot = URDF.from_parameter_server()
		
		# Now point output back
		sys.stdout = sys.__stdout__
		sys.stderr = sys.__stderr__
		devnull.close()

		self.joint_list = {}
		for ndx, jnt in enumerate(self.robot.joints):
			self.joint_list[jnt.name] = ndx

		self.chain = list()

		# Query parameter server for joints
		arm_chain = '/' + side + '_arm_chain'
		joint_names = rospy.get_param(arm_chain)

		for joint in joint_names:
			self.chain.append(JointData(self, joint))


class JointData(object):

	def __init__(self, main, name):
		self.main     = main
		self.name     = name

if __name__ == "__main__":
	main()