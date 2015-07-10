#!/usr/bin/env python

import rospy

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from flor_grasp_msgs.msg import GraspSelection

class IRobotReleaseGraspState(EventState):
    
        
    LEFT_HAND = 'l_hand'
    RIGHT_HAND = 'r_hand'
    def __init__(self, hand):
        '''-
        Constructor
        '''
        super(IRobotReleaseGraspState,self).__init__(outcomes=['done'])
        self._controller_topic = '/grasp_control/' + hand + '/release_grasp'
        self._pub = ProxyPublisher({self._controller_topic : GraspSelection})
        
    def execute(self, userdata):
        return 'done'
    
    def on_enter(self, userdata):
        message = GraspSelection()
        message.header.stamp = rospy.Time.now()
        self.loginfo('Releasing Grasp')
        self._pub.publish(self._controller_topic, message)
        
        
