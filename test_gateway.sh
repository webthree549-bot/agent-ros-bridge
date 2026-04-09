#!/bin/bash
# Test connection to Agent ROS Bridge Gateway
unset HTTPS_PROXY HTTP_PROXY ALL_PROXY
python3 ~/.openclaw/workspace/openclaw_gateway_client.py
