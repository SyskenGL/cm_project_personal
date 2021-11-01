#!/usr/bin/env python3
from cm_ros.wrapper import CMNode


class CMLEDs(CMNode):

    def __init__(self, name='cm_leds'):
        CMNode.__init__(self, name)

    def run(self):
        pass