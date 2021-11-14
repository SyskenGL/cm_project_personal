#!/usr/bin/env python3
from cm_bridge.touch_sensors import CMTouchSensors


if __name__ == "__main__":
    cm_touch_sensors = CMTouchSensors()
    cm_touch_sensors.start()
