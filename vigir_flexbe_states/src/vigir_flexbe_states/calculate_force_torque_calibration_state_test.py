# Test the FT Calibration state by calling the python class and doing the calculation here

# Moritz Schappler, schappler@irt.uni-hannover.de, 2015-05
# Institut fuer Regelungstechnik, Universitaet Hannover


# remotedebug
# import pydevd
# pydevd.settrace('localhost', port=5678, stdoutToServer=True, stderrToServer=True)


# import definitions
from calculate_force_torque_calibration_state import CalculateForceTorqueCalibration
from generate_trajectory_from_txtfile_state import GenerateTrajectoryFromTxtfileState

# initialize rospy and rostime
import rospy  
rospy.init_node('calib_test_node', anonymous=True)

# define userdata
class Userdata(object):
    def __init__(self):
        self.trajectories = []
        self.ft_calib_data = []



# Generating the trajectory from text files
# txtfile_name_left_arm = '~/ft_calib/input/l_arm.txt'
# txtfile_name_right_arm = '~/ft_calib/input/r_arm.txt'
txtfile_name_left_arm = '~/ft_calib/input/SI_E065_FT_Calib_Arms_Payload_Left.txt'
txtfile_name_right_arm = '~/ft_calib/input/SI_E065_FT_Calib_Arms_Payload_Right.txt'
transitiontime = 0.5
settlingtime = 0.5

userdata = Userdata()
GTFT = GenerateTrajectoryFromTxtfileState(txtfile_name_left_arm, txtfile_name_right_arm, transitiontime, settlingtime)
GTFT.execute(userdata)

# Calculation the calibration with data recorded with the behaviour
# bag_filename = '/home/schappler/ft_calib/ft_logs/FTCalib.bag'
# bag_filename = '/home/schappler/ft_calib/ft_logs/R05_both_20150426_w_flangue.bag'
# bag_filename = '/home/schappler/IRT_DRC/Experiments/Output/SI_E047_FT_Calib_Arms/S02_20150504_payload_merge.bag'
bag_filename = '~/ft_calib/ft_logs/SI_E065_FT_Calib_Arms_Payload.bag'



calibration_chain = ['left_arm', 'right_arm']
trajectories_command = GTFT._trajectories

CFTC = CalculateForceTorqueCalibration(bag_filename, calibration_chain, settlingtime, trajectories_command)
CFTC.execute(userdata)
print CFTC._ft_calib_data
