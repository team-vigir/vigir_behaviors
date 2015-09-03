#!/usr/bin/env python

import os
import math

import rospy

from flexbe_core import EventState, Logger
from vigir_flexbe_states.proxy import ProxyMoveitClient
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

from vigir_object_template_msgs.msg import *
from vigir_object_template_msgs.srv import *
from geometry_msgs.msg import Point

'''
Created on 04/09/2015

@author: Philipp Schillinger and Spyros Maniatopoulos
'''
class PlanAffordanceState(EventState):
    '''
    Plans for a given affordance without collision avoidance.

    -- vel_scaling             float                Scales the velocity of the planned trajectory,
                                                    lower values for slower trajectories.
    -- planner_id              string               Sets the ID of the planner to use (MoveIt planner id or "drake" - default = "RRTConnectkConfigDefault")
    -- drake_sample_rate       float                Sample rate for Drake's result trajectory (in Hz, default = 4.0)
    -- drake_orientation_type  int                  How should the eef-orientation be handled (see ExtendedPlanningOptions, 0 = full, 1 = axis only, 2 = ignore)
    -- drake_link_axis         float[]              Target axis to keep (only for drake_orientation_type == ORIENTATION_AXIS_ONLY

    ># affordance           Affordance          A message as defined in vigir_object_template_msgs containing all required information.
    ># hand                 string              The hand which should execute the given affordance.
    ># reference_point      PoseStamped         A point of interest on the template w.r.t.
                                                which planning will take place (defaults to palm if not set).
                                                A template's 'usability' pose would go here.

    #> joint_trajectory     JointTrajectory     Trajectory of the end effector to perform the requested motion.
    #> plan_fraction        float               Fraction of the requested motion which could be planned.
                                                A value below 1 indicates a partial motion.

    <= done                                     Successfully planned the given affordance.
    <= failed                                   Failed to plan the affordance.

    '''

    LEFT_HAND = 'left'
    RIGHT_HAND = 'right'
    
    def __init__(self, vel_scaling = 0.1, planner_id = "RRTConnectkConfigDefault", drake_sample_rate = 4.0, drake_orientation_type = 2, drake_link_axis = [0, 0, 1]):
        '''Constructor'''
        super(PlanAffordanceState, self).__init__(outcomes = ['done', 'incomplete', 'failed'],
                                                  input_keys = ['affordance', 'hand', 'reference_point'],
                                                  output_keys = ['joint_trajectory', 'plan_fraction'])

        self._client = ProxyMoveitClient()

        try:
            self.set_up_affordance_service() # topics and clients are set here
        except Exception as e:
            Logger.logerr('Failed to set up GetAffordanceInWristFrame service: \n%s' % str(e))
            return

        self._vel_scaling = vel_scaling
        self._planner_id = planner_id
        
        self._drake_sample_rate = drake_sample_rate
        self._drake_orientation_type = drake_orientation_type
        self._drake_link_axis = Point( x = drake_link_axis[0], y = drake_link_axis[1], z = drake_link_axis[2] )

        self._failed = False
        self._done = False
        self._incomplete = False


    def execute(self, userdata):
        '''Wait for result and determine outcome based on fraction thresholds'''

        fail_threshold = 0.05
        success_threshold = 0.95

        if self._failed:
            return 'failed'
        if self._done:
            return 'done'
        if self._incomplete:
            return 'incomplete'

        if self._client.finished():
            userdata.joint_trajectory = self._client.get_plan()
            
            if self._client.success():
                plan_fraction = self._client.get_plan_fraction()
                Logger.loginfo('Plan completion fraction = %f%%' % (plan_fraction*100))
                userdata.plan_fraction = plan_fraction

                if plan_fraction < fail_threshold:
                    Logger.logwarn('Failure! The plan fraction was below %d%%' % (fail_threshold*100))
                    self._failed = True
                    return 'failed'
                elif plan_fraction > success_threshold:
                    Logger.loginfo('Success! The plan fraction was above %d%%' % (success_threshold*100))
                    self._done = True
                    return 'done'
                else:
                    Logger.logwarn('Partial success.The plan fraction was between %d%% and %d%%' % (fail_threshold*100, success_threshold*100))
                    self._incomplete = True
                    return 'incomplete'
            
            else:
                Logger.logwarn('MoveIt failed with the following error: %s' % self._client.error_msg())
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        '''Handle cartesian/circular affordance, set options, and send goal.'''

        self._failed = False
        self._done = False
        self._incomplete = False

        planning_group_str = 'l_arm_group' if userdata.hand == PlanAffordanceState.LEFT_HAND else 'r_arm_group'
        
        affordance = userdata.affordance

        print affordance #TODO: remove temp debug statement

        # Create the manipulation action goal
        self._client.new_goal(move_group = planning_group_str)

        if affordance.type == "circular":
            self._client.set_circular_motion(angle = affordance.displacement,
                                             pitch = affordance.pitch)
            
            for waypoint in affordance.waypoints: # There should only be one
                self._client.add_endeffector_pose(waypoint.pose,
                                                  waypoint.header.frame_id)

                print waypoint #TODO: remove temp debug statement
        
        elif affordance.type == "cartesian":
            self._client.set_cartesian_motion()

            try:
                srv_request = GetAffordanceInWristFrameRequest(affordance)
                srv_result = self._srv.call(self._service_topics[userdata.hand],
                                            srv_request)
                wrist_affordance = srv_result.wrist_affordance # PoseStamped
                
                print wrist_affordance #TODO: remove temp debug statement

                self._client.add_endeffector_pose(wrist_affordance.pose,
                                                  wrist_affordance.header.frame_id)

            except Exception as e:
                Logger.logwarn('Failed to transform affordance to wrist frame: \n%s' % str(e))
                self._failed = True
                return
        else:
            Logger.logwarn("Unknown affordance type: %s" % affordance.type)
            self._failed = True
            return

        # Set planning options
        self._client.set_planner_id(self._planner_id)
        self._client.set_velocity_scaling(self._vel_scaling)
        self._client.set_collision_avoidance(True)
        self._client.set_execute_incomplete_plans(True)
        self._client.set_keep_orientation(affordance.keep_orientation)
        
        #Set reference_point and frame, if provided. Defaults to palm.
        if userdata.reference_point is not None:
            self._client.set_reference_point(
                        pose = userdata.reference_point.pose,
                        frame_id = userdata.reference_point.header.frame_id)
                
        #Set Drake specific options        
        self._client.set_drake_target_orientation_type(self._drake_orientation_type)
        self._client.set_drake_target_link_axis(self._drake_link_axis)
        self._client.set_drake_trajectory_sample_rate(self._drake_sample_rate)        
  
        try:
            self._client.start_planning()
        except Exception as e:
            Logger.logwarn('Could not request a plan for the affordance!\n%s' % str(e))
            self._failed = True


    def on_exit(self, userdata):
        if not self._client.finished():
            self._client.cancel()
            Logger.loginfo("Cancelled active action goal.")

    ############################################################################

    def set_up_affordance_service(self):
        
        hands_in_use = self.determine_hands_in_use()

        # Set up services for transforming cartesian affordances to wrist frame
        self._service_topics = dict()

        for hand in hands_in_use:

            topic = '/manipulation_control/%s_hand/wrist_affordance_srv' % hand[0]
            self._service_topics[hand] = topic
        
        self._srv = ProxyServiceCaller(
            {t: GetAffordanceInWristFrame for t in self._service_topics.values()})


    def determine_hands_in_use(self):
        '''Determine which hand types and sides have been sourced'''

        if not rospy.has_param("behavior/hand_type_prefix"):
            Logger.logerr("Need to specify behavior/parameter hand_type_prefix at the parameter server")
            return

        hand_type_prefix = rospy.get_param("behavior/hand_type_prefix")

        hands_in_use = list()

        LEFT_HAND  = os.environ[hand_type_prefix + 'LEFT_HAND_TYPE']
        RIGHT_HAND = os.environ[hand_type_prefix + 'RIGHT_HAND_TYPE']

        if LEFT_HAND not in ['l_stump', 'l_hook']:
            hands_in_use.append('left')
        if RIGHT_HAND not in ['r_stump', 'r_hook']:
            hands_in_use.append('right')

        if len(hands_in_use) == 0:
            Logger.logerr('No hands seem to be in use:\nLEFT_HAND = %s\nRIGHT_HAND = %s' % (LEFT_HAND, RIGHT_HAND))

        return hands_in_use
