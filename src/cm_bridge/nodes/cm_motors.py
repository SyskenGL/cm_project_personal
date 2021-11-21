#!/usr/bin/env python3
from cm_bridge.motors import CMMotors


if __name__ == "__main__":
    cm_microphone = CMMotors()
    cm_microphone.start()
