#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode


class CMAPIManager(CMNode):

    def __init__(self, name='cm_motors'):
        CMNode.__init__(self, name)

    def run(self):
        pass