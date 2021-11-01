#!/usr/bin/env python3
from cm_bridge.leds import CMLEDs


if __name__ == '__main__':
    cm_leds = CMLEDs()
    cm_leds.start()