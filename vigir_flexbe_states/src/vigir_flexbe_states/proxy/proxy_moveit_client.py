#!/usr/bin/env python

import rospy
import actionlib
from threading import Timer
import time
import sys

import moveit_commander
from moveit_msgs.msg import *
from vigir_planning_msgs.msg import *

from flexbe_core.logger import Logger
from flexbe_core.proxy import ProxyActionClient


class ProxyMoveitClient(object):
    """
    A proxy for easily using moveit.
    """
    _is_initialized = False

    _action_topic = "/vigir_move_group"

    _robot = None
    _client = None

    def __init__(self):
        if not ProxyMoveitClient._is_initialized:
            ProxyMoveitClient._is_initialized = True
            Logger.loginfo("Initializing proxy MoveIt client...")

            moveit_commander.roscpp_initialize(sys.argv)
            ProxyMoveitClient._robot = moveit_commander.RobotCommander()
            ProxyMoveitClient._client = ProxyActionClient({ProxyMoveitClient._action_topic: MoveAction})

        self._goal = None
        self._move_group = ""
        self._result = None
        self._planner_id = "RRTConnectkConfigDefault"


    def new_goal(self, move_group):
        self._move_group = move_group
        self._result = None
        self._goal = MoveGoal()
        self._goal.request.group_name = self._move_group
        self._goal.request.allowed_planning_time = 5.0
        self._goal.request.num_planning_attempts = 1
        self._goal.request.planner_id = self._planner_id
        self._goal.extended_planning_options.target_motion_type = ExtendedPlanningOptions.TYPE_FREE_MOTION

    def start_planning(self):
        self._goal.planning_options.plan_only = True
        ProxyMoveitClient._client.send_goal(ProxyMoveitClient._action_topic, self._goal)
        self._goal = None

    def start_execution(self):
        ProxyMoveitClient._client.send_goal(ProxyMoveitClient._action_topic, self._goal)
        self._goal = None


    def set_replan(self, value):
        self._goal.planning_options.replan = value

    def set_velocity_scaling(self, value):
        self._goal.request.max_velocity_scaling_factor = value

    def set_collision_avoidance(self, value):
        self._goal.extended_planning_options.allow_environment_collisions = value

    def set_execute_incomplete_plans(self, value):
        self._goal.extended_planning_options.execute_incomplete_cartesian_plans = value

    def set_keep_orientation(self, value):
        self._goal.extended_planning_options.keep_endeffector_orientation = value

    def set_cartesian_motion(self):
        self._goal.extended_planning_options.target_motion_type = ExtendedPlanningOptions.TYPE_CARTESIAN_WAYPOINTS

    def set_circular_motion(self, angle, pitch):
        self._goal.extended_planning_options.target_motion_type = ExtendedPlanningOptions.TYPE_CIRCULAR_MOTION
        self._goal.extended_planning_options.rotation_angle = angle
        self._goal.extended_planning_options.pitch = pitch # meters

    def set_reference_point(self, pose, frame_id):
        self._goal.extended_planning_options.reference_point = pose # Pose msg
        self._goal.extended_planning_options.reference_point_frame = frame_id

    def set_allowed_collision(self, link, target):
        self._goal.extended_planning_options.extended_planning_scene_diff.allowed_collision_pairs.collision_entities = [link, target]
        
    def set_planner_id(self, planner_id):
        self._planner_id = planner_id
        self._goal.request.planner_id = planner_id        

    def add_joint_values(self, joint_values):
        joint_names = ProxyMoveitClient._robot.get_joint_names(self._move_group)
        if len(joint_names) != len(joint_values):
            Logger.logwarn("Amount of given joint values (%d) does not match the amount of joints (%d)." % (len(joint_values), len(joint_names)))
            #return

        goal_constraints = Constraints()
        for i in range(len(joint_names)):
            goal_constraints.joint_constraints.append(JointConstraint(joint_name=joint_names[i], position=joint_values[i]))

        self._goal.request.goal_constraints.append(goal_constraints)

    def add_endeffector_pose(self, target_pose, frame_id = "/world"):
        self._goal.extended_planning_options.target_frame = frame_id
        self._goal.extended_planning_options.target_poses.append(target_pose)

    def add_link_padding(self, link_paddings):
        for link_name, padding in link_paddings.iteritems():
            link_padding = LinkPadding(link_name = link_name, padding = padding)
            self._goal.planning_options.planning_scene_diff.link_padding.append(link_padding)

    def finished(self):
        if ProxyMoveitClient._client.has_result(ProxyMoveitClient._action_topic):
            self._result = ProxyMoveitClient._client.get_result(ProxyMoveitClient._action_topic)
            return True
        return False

    def success(self):
        return self._result.error_code.val == MoveItErrorCodes.SUCCESS

    def error_msg(self):
        for key, value in MoveItErrorCodes.__dict__.items():
            if value == self._result.error_code.val and key[0] != "_":
                return key + (" (%s)" % str(self._result.error_code.val))
        return "unknown error (%s)" % str(self._result.error_code.val)

    def cancel(self):
        ProxyMoveitClient._client.cancel(ProxyMoveitClient._action_topic)

    def get_plan(self):
        return self._result.planned_trajectory.joint_trajectory

    def get_plan_fraction(self):      
        return self._result.extended_planning_result.plan_completion_fraction
