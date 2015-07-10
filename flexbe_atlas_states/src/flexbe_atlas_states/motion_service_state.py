#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxyActionClient

from motion_editor_core.srv import ExecuteMotion, ExecuteMotionRequest, ExecuteMotionResponse
from actionlib import SimpleActionClient
from motion_editor_core.msg import ExecuteMotionAction, ExecuteMotionGoal


"""
Created on 30.05.2013

@author: Philipp Schillinger, Martin Oehler
"""


class MotionServiceState(EventState):
    """Implements a state where a certain motion is performed.
    This state can be used to execute motions created by the motion editor.

    -- motion_key 	string 	Reference to the motion to be executed.
    -- time_factor	float	Factor to multiply with the default execution time of the specified motion.
                            For example, 2 will result in a motion twice as long, thus executed at half speed.

    <= done 				Indicates that the expected execution time of the motion has passed.
                            The motion service provides no way to check if a motion has actually finished.
    <= failed               Indicates that the requested motion doesn't exist and therefore wasn't executed
    """

    def __init__(self, motion_key, time_factor=1):
        """
        Constructor
        """
        super(MotionServiceState, self).__init__(outcomes=['done', 'failed'])

        self.motion_key = motion_key
        self.time_factor = time_factor
        self._finish_time = None
        self._motion_goal_ns = '/motion_service/motion_goal'

        self._client = ProxyActionClient({self._motion_goal_ns: ExecuteMotionAction})

        self._failed = False
        self._done = False

    def execute(self, userdata):
        """
        Execute this state
        """
        if self._failed:
            return 'failed'
        if self._done:
            return 'done'

        if self._client.has_result(self._motion_goal_ns):
            result = self._client.get_result(self._motion_goal_ns)
            if result is None:  # Bug in actionlib, sometimes returns None instead of result
                # Operator decision needed
                Logger.logwarn("Failed to execute the motion '%s':\nAction result is None" % self.motion_key)
                self._failed = True
                return 'failed'
            if result.error_string is None or len(result.error_code) == 0:
                Logger.logwarn("Failed to execute the motion '%s':\nAction result is None" % self.motion_key)
                self._failed = True
                return 'failed'

            success = all(lambda x: x == 0, result.error_code)

            if success:
                rospy.loginfo('Trajectory finished successfully.') # dont need to send this to the operator
                self._done = True
                return 'done'
            else:
                Logger.logwarn("Failed to execute the motion '%s':\n%s" % (self.motion_key, str(result.error_code)))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        self._failed = False
        self._done = False

        # build goal
        goal = ExecuteMotionGoal()
        goal.motion_name = self.motion_key
        goal.time_factor = self.time_factor

        try:
            self._client.send_goal(self._motion_goal_ns, goal)
        except Exception as e:
            Logger.logwarn("Failed sending motion goal for '%s':\n%s" % (self.motion_key, str(e)))

    def on_exit(self, userdata):
        if not self._client.has_result(self._motion_goal_ns):
            self._client.cancel(self._motion_goal_ns)
            Logger.loginfo("Cancelled active motion goal.")
