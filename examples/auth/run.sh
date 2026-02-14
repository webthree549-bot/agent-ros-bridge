#!/bin/bash
# Authentication Example - JWT Security

set -e

echo "🔐 Starting Authentication Demo"
echo "================================="
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "mock_bridge_auth.py" ]; then
    echo "❌ Run this script from the auth/ directory"
    exit 1
fi

echo "Starting authenticated bridge on ws://localhost:8768"
echo ""
echo "JWT Secret: my-secret-key (demo only!)"
echo ""
echo "Generate a token:"
echo "  python ../../scripts/generate_token.py --secret my-secret-key --role operator"
echo ""
echo "Connect with token:"
echo "  wscat -c 'ws://localhost:8768?token=YOUR_TOKEN'"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 mock_bridge_auth.py "$@"
