#!/usr/bin/env python3
from cm_bridge.api_manager import CMAPIManager


if __name__ == '__main__':
    cm_api_manager = CMAPIManager()
    cm_api_manager.start()