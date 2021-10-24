#!/usr/bin/env python3
from cm_bridge.serial_manager import CMSerialManager


if __name__ == '__main__':
    cm_serial_manager = CMSerialManager()
    cm_serial_manager.start()