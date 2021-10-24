#!/usr/bin/env python3
from cm_ros.wrapper import CMNode


class CMSerialManager(CMNode):

    def __init__(self, name='cm_serial_manager'):
        CMNode.__init__(self, name)

    def run(self):
        print("test")