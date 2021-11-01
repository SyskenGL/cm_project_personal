#!/usr/bin/env python3
from cm_bridge.api_manager import CMMotors


if __name__ == '__main__':
    cm_motors = CMMotors()
    cm_motors.start()