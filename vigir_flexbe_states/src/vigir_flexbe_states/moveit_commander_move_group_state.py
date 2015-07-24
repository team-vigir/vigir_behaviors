#!/usr/bin/env python

from flexbe_core import EventState, Logger
from moveit_commander import MoveGroupCommander

"""
Created on 03/06/2015

@author: Alexander Spitzer
"""

class MoveitCommanderMoveGroupState(EventState):
  """
  Uses moveit commander to plan to the given joint configuration and execute the resulting trajctory.
  """
  
  def __init__(self, planning_group, joint_names):
    """Constructor"""
    super(MoveitCommanderMoveGroupState, self).__init__(outcomes=['reached', 'failed'],
                    input_keys=['target_joint_config'])

    self._planning_group = planning_group
    self._joint_names = joint_names
    Logger.loginfo("About to make mgc in init with group %s" % self._planning_group)
    self.group_to_move = MoveGroupCommander(self._planning_group)
    Logger.loginfo("finished making mgc in init.")

    self._done = False


  def execute(self, userdata):
    """Execute this state"""
    if self._done is not False:
      return self._done
  
  def on_enter(self, userdata):
    # create the motion goal
    Logger.loginfo("Entering MoveIt Commander code!")

    if len(self._joint_names) != len(userdata.target_joint_config):
      Logger.logwarn("Number of joint names (%d) does not match number of joint values (%d)"
                      % (len(self._joint_names), len(userdata.target_joint_config)))

    self.group_to_move.set_joint_value_target(dict(zip(self._joint_names, userdata.target_joint_config)))               


    # execute the motion
    try: 
      Logger.loginfo("Moving %s to: %s" % (self._planning_group, ", ".join(map(str, userdata.target_joint_config))))
      result = self.group_to_move.go()
    except Exception as e:
      Logger.logwarn('Was unable to execute move group request:\n%s' % str(e))
      self._done = "failed"

    if result:
      self._done = "reached"
    else:
      self._done = "failed"
