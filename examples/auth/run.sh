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

if [ ! -f "auth_demo.py" ]; then
    echo "❌ Run this script from the auth/ directory"
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
echo "Starting authenticated bridge on ws://localhost:8768"
echo ""
echo "Generate a token:"
echo "  python ../../scripts/generate_token.py --secret \$JWT_SECRET --role operator"
echo ""
echo "Connect with token:"
echo "  wscat -c 'ws://localhost:8768?token=YOUR_TOKEN'"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 auth_demo.py "$@"
