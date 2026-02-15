#!/bin/bash
# Quickstart Example - Simulated Robot Bridge

set -e

echo "🚀 Starting Agent ROS Bridge - Quickstart"
echo "=========================================="
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

# Check if running from correct directory
if [ ! -f "simulated_robot.py" ]; then
    echo "❌ Run this script from the quickstart/ directory"
    exit 1
fi

# SECURITY: Check for JWT_SECRET
if [ -z "$JWT_SECRET" ]; then
    echo "⚠️  SECURITY WARNING: JWT_SECRET not set"
    echo ""
    echo "This example requires authentication. Generate a secret:"
    echo "  export JWT_SECRET=\$(openssl rand -base64 32)"
    echo ""
    exit 1
fi

echo "✅ JWT_SECRET configured"
echo "Starting simulated robot bridge on ws://localhost:8765"
echo ""
echo "Generate a token to connect:"
echo "  python ../../scripts/generate_token.py --secret \$JWT_SECRET --role operator"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 simulated_robot.py "$@"
