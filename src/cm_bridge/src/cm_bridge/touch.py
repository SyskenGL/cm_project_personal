#!/usr/bin/env python3
from cm_ros.wrapper import CMNode


class CMTouch(CMNode):

    def __init__(self, name='cm_touch'):
        CMNode.__init__(self, name)

