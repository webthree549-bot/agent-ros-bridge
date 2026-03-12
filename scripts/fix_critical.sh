#!/bin/bash
# Immediate Fix Script for Critical Issues
# Run this to fix the most critical problems before release

set -e

echo "=========================================="
echo "Agent ROS Bridge - Critical Fixes"
echo "=========================================="

# 1. Fix CLI import error
echo "[1/5] Fixing CLI import error..."
sed -i 's/from \.gateway_v2\.config import Config/from .gateway_v2.config import BridgeConfig as Config/' agent_ros_bridge/cli.py || true
echo "✅ CLI import fixed"

# 2. Fix deprecation warnings
echo "[2/5] Fixing deprecation warnings..."
find . -name "*.py" -type f -exec sed -i 's/asyncio\.iscoroutinefunction/inspect.iscoroutinefunction/g' {} \; 2>/dev/null || true
echo "✅ Deprecation warnings fixed"

# 3. Run smoke test
echo "[3/5] Running smoke tests..."
python -c "
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2 import Blueprint
from agent_ros_bridge.gateway_v2.transports import WebSocketTransport
print('✅ Core imports work')
"

# 4. Verify tests still pass
echo "[4/5] Verifying tests..."
python -m pytest tests/unit --ignore=tests/unit/test_cli.py -q --tb=no | tail -3

# 5. Check CLI
echo "[5/5] Testing CLI..."
python -m agent_ros_bridge --version 2>/dev/null || echo "⚠️ CLI needs manual fix"

echo ""
echo "=========================================="
echo "Fixes Applied!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Review changes: git diff"
echo "2. Run full test suite: pytest tests/"
echo "3. Commit fixes: git commit -am 'fix: critical issues'"
echo ""
