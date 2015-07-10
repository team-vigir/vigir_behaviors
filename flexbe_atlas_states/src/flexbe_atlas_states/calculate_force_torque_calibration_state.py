#!/usr/bin/env python

import rospy
import numpy as np # http://docs.scipy.org/doc/numpy/reference/routines.html
from copy import copy
import tf
import os
from flexbe_core import EventState, Logger
from rosbag import Bag


"""
Created on 01/05/2015

@author: Moritz Schappler

Sources: 
[1] Kroenig: Kraft-/Drehmomentsensoren-Kalibrierung

TODOs:
Move chain loop inside the opening of the bag file
"""

class CalculateForceTorqueCalibration(EventState):
	"""
	Calculate a Calibration Dataset for the force torque sensor

	-- calibration_chain 			string[]	Names of the chain of the ft sensors to be calibrated.
	-- settlingtime		float		settling time after each pose
	-- 	static_calibration_data	dict()	dictionary, where keys are the entries of calibration_chain and elements are lists with 4 entries: Mass, center of mass from previous calibrations
	
	#> trajectories_command JointTrajectory{}	A dictionary where the keys are ['left_arm', 'right_arm', ...] and each has a trajectory as the value.
	#> bag_filename		 			string[]	path to the bag file to be read for calibration

	># ft_calib_data 	float[] 	calibration data for the given ft sensors.

	<= done 						Successfully calculated the calibration of all ft sensors.
	<= failed 						error in the calculation.

	"""

	def __init__(self, calibration_chain, settlingtime, static_calibration_data):
		"""Constructor"""
		super(CalculateForceTorqueCalibration, self).__init__(outcomes=['done', 'failed'],
												input_keys=['bag_filename', 'trajectories_command'], 
												output_keys=['ft_calib_data'])

		self._failed = False
		self._done = False
		
		# inputs
		self._calibration_chain = calibration_chain
		self._settlingtime = settlingtime
		self._static_calibration_data = static_calibration_data

		# prepare tmpp variable for output
		self._ft_calib_data = dict() # dictionary like userdata.ft_calib_data
		for chain in calibration_chain:
			self._ft_calib_data[chain] = np.zeros((10,1))

	def execute(self, userdata):
		if self._failed:
			return 'failed'
		if self._done:
			return 'done'

	def on_enter(self, userdata):
		self._failed = False
		try:
			# Initialization
		
			# userdata
			self._bag_filename = userdata.bag_filename
			self._trajectories_command = userdata.trajectories_command
			
			# joint definitions
			l_arm_range = range(16,23);
			r_arm_range = range(23,30);
			atlasJointNames = [
			    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
			    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
			    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
			    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2',
			    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']
			joint_position_cmd = [0]*len(l_arm_range)
			# starting and stopping time of measurement data to be taken into account for cailbration
			time_start = 0
			time_end = 0
			time_tf = 0
			
			# gravity vector in world frame
			g_world = np.resize(np.array([0,0,-9.81]),(3,1))
		
			# take this number of data points for each pose.
			max_data_points_per_pose = 100
			
			# rotation of the force torque sensor in world frame (quaternions)
			ft_rotation = [0,0,0,0]
			
			# loop over all ft sensors to calibrate
			
			for chain in self._calibration_chain: 
				tf_chain=['/pelvis', 'ltorso', 'mtorso', 'utorso']
				if chain == 'left_arm':
					joint_range = l_arm_range
					tfname = 'l_hand'
					tf_chain.extend(['l_clav', 'l_scap', 'l_uarm', 'l_larm', 'l_ufarm', 'l_lfarm', 'l_hand'])
				elif chain == 'right_arm':
					joint_range = r_arm_range
					tf_chain.extend(['r_clav', 'r_scap', 'r_uarm', 'r_larm', 'r_ufarm', 'r_lfarm', 'r_hand'])
					
					tfname = 'r_hand'
				else:
					Logger.logwarn('CalculateForceTorqueCalibration: Undefined chain %s', chain)

				# initialize transformations
				tf_data = [] # frame to frame transformation with numerical indexes from the tf_chain
				tf_data_cum = [] # world to frame transformation with numerical indexes from the tf_chain

				for i in range(len(tf_chain)):
					tf_data.append([0,0,0,0])
					tf_data_cum.append([0,0,0,0])
									
				# get number of poses from the commanded trajectories. 2 Points per Pose
				number_of_poses = len(self._trajectories_command[chain].points) / 2
				# define information matrix for calibration
				InfMat = np.zeros((6*max_data_points_per_pose*number_of_poses, 10))
				InfMat_i = 0 # Index in this Matrix
				# definie measurement vector for calibration
				MeasVec = np.zeros((6*max_data_points_per_pose*number_of_poses, 1))
				
				# commanded joint trajectory for current chain
				# all commanded positions in this trajectory
				current_traj_cmd = self._trajectories_command[chain] 
				
				# flag if the time interval was already set:
				timesetflag = False
				
				point_index_cmd = 0 # start with index 0
				transformation_available = False
				
				# read time series from bag file
				# check in desired input trajectory, at which time a position is reached
				# take the data in a defined time period after the commanded new position
				bag_from_robot = Bag(os.path.expanduser(self._bag_filename)) # open bag file
				Logger.loginfo('CalculateForceTorqueCalibration: Calibrate %s from %s. Using %d different positions from trajectory command. Expecting 2 commanded positions per pose' % (chain, self._bag_filename, len(current_traj_cmd.points)) )
				for topic, msg, t in bag_from_robot.read_messages(topics=['/flor/controller/atlas_state', '/tf', '/flor/controller/joint_command']):
					# Check if all desired poses have been reached
					if point_index_cmd > len(current_traj_cmd.points)-1:
						break
					current_position_cmd = current_traj_cmd.points[point_index_cmd].positions
					####################################################
					# Check tf message: remember last transformation from world to ft sensor
					if topic == '/tf':
			  			# loop over all transformations inside this tf message
						for j in range(len(msg.transforms)):
							data = msg.transforms[j]
							tr = data.transform # current transformation
							header = data.header
							# check if frame matches on of the saved frames
							for i in range(len(tf_chain)):
								if data.child_frame_id == tf_chain[i]:
									tf_data[i] = np.array([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])

						# calculate the newest transformation to the force torque sensor
						tf_data_cum[0] = tf_data[0]
						for i in range(1,len(tf_chain)):
							tf_data_cum[i] = q = tf.transformations.quaternion_multiply(tf_data_cum[i-1], tf_data[i])
							if i == len(tf_chain)-1:
								time_tf = msg.transforms[0].header.stamp.to_sec()
						# nothing to do with tf message
						# check if all transformations are available. (up to the last link)
						if np.any(tf_data_cum[-1]): # real part of quaternion unequal to zero: data exists
							transformation_available = True
						continue
					else:
						time_msg = msg.header.stamp.to_sec()
					
					####################################################
					# check if timestamp is interesting. Then check the 
					if not (time_msg > time_start and time_msg < time_end) and not timesetflag:
						# the timestamp is not in evaluation interval. Look for reaching of the pose
						# record data (see below)
						if topic == '/flor/controller/joint_command':
				  			# get the commanded position
							for i in range(len(joint_range)):
								osrf_ndx = joint_range[i]
								joint_position_cmd[i] = msg.position[osrf_ndx]
	
				  			# Check if position command matches the currently expected commanded position
				  			pos_reached = True

				  			for i in range(len(joint_range)):
				  				if abs(joint_position_cmd[i]-current_position_cmd[i]) > 0.001:
				  					pos_reached = False
				  					break
		  					if pos_reached:
			  					# end time of the values regarded for calibration data: take the latest time possible  for this pose
			  					time_end = msg.header.stamp.to_sec() + self._settlingtime
			  					# starting time for calibration: Take 100ms
			  					time_start = time_end - 0.1
			  					data_points_for_this_pose = 0
			  					timesetflag = True
			  					# take the next point next time. Each pose consists of two trajectory points (one for reaching, one for settling).
			  					# take the second one
			  					point_index_cmd = point_index_cmd + 2
				  			if not pos_reached:
				  				# The commanded position has not been reached. Skip
				  				continue
			  			continue # continue here, because data aquisition is triggered by the time_start, time_end
			  		
			  		if time_msg > time_end:
			  			timesetflag = False # prepare for new evaluation interval
			  			
					####################################################
					# check if enough datapoints for this pose have been collected
			  		if data_points_for_this_pose > max_data_points_per_pose:
			  			# already enough data points for this pose	
			  			continue
			  		
			  		####################################################
			  		# Check if message is atlas_state
			  		# IF this is the case, fill information matrix
			  		if topic != '/flor/controller/atlas_state':
			  			continue

					# Extract measured force and torque
					if chain == 'left_arm':
						FT = [msg.l_hand.force.x, msg.l_hand.force.y, msg.l_hand.force.z, msg.l_hand.torque.x, msg.l_hand.torque.y, msg.l_hand.torque.z ]
					elif chain == 'right_arm':
						FT = [msg.r_hand.force.x, msg.r_hand.force.y, msg.r_hand.force.z, msg.r_hand.torque.x, msg.r_hand.torque.y, msg.r_hand.torque.z ]
	
					# calculate gravitation vector
					if not transformation_available:
						Logger.logwarn('No tf messages available at time %1.4f.' % time_msg)
						continue
						
					R = tf.transformations.quaternion_matrix(tf_data_cum[-1])
					g = np.dot((R[0:3,0:3]).transpose(), g_world)
	
					gx = g[0]
					gy = g[1]
					gz = g[2]
				
					# fill information matrix for this data point. Source: [1], equ. (7)
					M = np.zeros((6, 10))
					M[0,0] = gx
					M[1,0] = gy
					M[2,0] = gz
					
					M[3,2] = gz
					M[3,3] = -gy 
					M[4,1] = -gz
					M[4,3] = gx
					M[5,1] = gy
					M[5,2] = -gx
					M[0,4] = 1.0
					M[1,5] = 1.0
					M[2,6] = 1.0
					M[3,7] = 1.0
					M[4,8] = 1.0
					M[5,9] = 1.0
					
					# fill big information matrix and vector (stack small information matrizes)
					for i in range(6):
						for j in range(10):
							InfMat[InfMat_i*6+i,j] = M[i,j]
						MeasVec[InfMat_i*6+i,0] = FT[i]
						
					InfMat_i = InfMat_i + 1 # increase index
					data_points_for_this_pose = data_points_for_this_pose + 1
			
				# shorten big information matrix
				if InfMat_i < max_data_points_per_pose*number_of_poses:
					InfMat_calc = InfMat[0:(6*InfMat_i)-1,:]
					MeasVec_calc = MeasVec[0:(6*InfMat_i-1),:]
				else:
					InfMat_calc = InfMat
					MeasVec_calc = MeasVec

				# calculate calibration data
				if chain in self._static_calibration_data.keys(): # calculate calibration with given static parameters
					# bring colums with first parameters on the other side of the equation
					if len(self._static_calibration_data[chain]) != 4:
						Logger.logwarn( "CalculateForceTorqueCalibration: Given static calibration data for %s has length %d. Required 4 entries." % (chain, len(k_fix)) )
						self._failed = True
						return
					# convert physical parameters to identification parameters (mass, 1st moment)
					m = self._static_calibration_data[chain][0]
					if m == 0:
						mom_x = 0
						mom_y = 0
						mom_z = 0
					elif m > 0:
						mom_x = self._static_calibration_data[chain][1]/m
						mom_y = self._static_calibration_data[chain][2]/m
						mom_z = self._static_calibration_data[chain][3]/m
					else:
						Logger.logwarn( "CalculateForceTorqueCalibration: Negative mass (%f) for calibration requested. Abort." % mass )
						self._failed = True
						return
					k_fix = np.resize(np.array([m, mom_x, mom_y, mom_z]),(4,1))
					Logger.loginfo( "CalculateForceTorqueCalibration:static calibration data for %s given: %s. Reduce equation system" % (chain, str(self._static_calibration_data[chain])) )
					MeasVec_calc_corr = np.subtract(np.array(MeasVec_calc), np.dot(InfMat_calc[:,0:4], k_fix))
					InfMat_calc_corr = InfMat_calc[:,4:10] # only the last 6 colums which correspond to the sensor offsets
					k = np.linalg.lstsq(InfMat_calc_corr, MeasVec_calc_corr)[0] # solve reduced equation system
					k_calibration = self._static_calibration_data[chain]
					k_calibration.extend(k)
				else: # calculate normally with all parameters unknown
					k = np.linalg.lstsq(InfMat_calc, MeasVec_calc)[0]
					# convert to physical parameters (first moment -> mass)
					if k[0] > 0:
						k_calibration = [k[0], k[1]/k[0], k[2]/k[0], k[3]/k[0], k[4], k[5], k[6], k[7], k[8], k[9]]
					else:
						Logger.loginfo("CalculateForceTorqueCalibration:Calibration brought negative mass %f" % k[0])
						k_calibration = [0.0, 0.0, 0.0, 0.0, k[4], k[5], k[6], k[7], k[8], k[9]]
					
				Logger.loginfo("CalculateForceTorqueCalibration:calibration data for %s" % chain)
				Logger.loginfo("CalculateForceTorqueCalibration:mass: %f" % float(k_calibration[0]))
				Logger.loginfo("CalculateForceTorqueCalibration:center of mass: %f %f %f" % (k_calibration[1], k_calibration[2], k_calibration[3]))
				Logger.loginfo("CalculateForceTorqueCalibration:F offset: %f %f %f" % (k_calibration[4], k_calibration[5], k_calibration[6]))
				Logger.loginfo("CalculateForceTorqueCalibration:M offset: %f %f %f" % (k_calibration[7], k_calibration[8], k_calibration[9]))
				self._ft_calib_data[chain] = k_calibration
			
				bag_from_robot.close()
	
			userdata.ft_calib_data = self._ft_calib_data
			Logger.loginfo('CalculateForceTorqueCalibration:Calibration finished')
			self._done = True

		except Exception as e:
			Logger.logwarn('CalculateForceTorqueCalibration:Unable to calculate calibration:\n%s' % str(e))
			self._failed = True
