#!/bin/bash
# Test connection to Agent ROS Bridge Gateway - clean environment
env -i PATH="$PATH" HOME="$HOME" python3 ~/.openclaw/workspace/openclaw_gateway_client.py
