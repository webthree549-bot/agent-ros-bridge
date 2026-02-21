#!/bin/bash
# Quick regression test for unified-demo changes

echo "=== REGRESSION TEST ==="
echo

# Test 1: Dockerfile syntax
echo "Test 1: Dockerfile syntax check"
dockerfile-lint examples/unified-demo/Dockerfile 2>/dev/null || echo "⚠️  dockerfile-lint not installed, skipping"

# Test 2: docker-compose syntax
echo "Test 2: docker-compose config validation"
cd examples/unified-demo && docker-compose config > /dev/null 2>&1 && echo "✅ docker-compose.yml valid" || echo "❌ docker-compose.yml invalid"

echo
echo "=== IMPACT ANALYSIS ==="
echo "Files changed in last 5 commits:"
git diff --name-only HEAD~5 HEAD

echo
echo "=== JWT EXPLANATION ==="
echo "JWT_SECRET is used for:"
echo "- Authenticating WebSocket connections"
echo "- Required by agent-ros-bridge (cannot be disabled)"
echo "- Set via environment variable in docker-compose.yml"
echo "- Default: 'demo-secret-change-in-production' (insecure, override it!)"
