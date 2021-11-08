#!/usr/bin/env python3
from cm_bridge.ir_sensors import CMIRSensors


if __name__ == '__main__':
    cm_ir_sensors = CMIRSensors()
    cm_ir_sensors.start()