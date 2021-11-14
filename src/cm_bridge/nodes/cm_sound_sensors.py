#!/usr/bin/env python3
from cm_bridge.sound_sensors import CMSoundSensors


if __name__ == "__main__":
    cm_sound_sensors = CMSoundSensors()
    cm_sound_sensors.start()
