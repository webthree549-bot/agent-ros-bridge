#!/bin/bash
# run_native.sh - Run unified demo directly on macOS without Docker

cd "$(dirname "$0")"

echo "🚀 Starting Agent ROS Bridge Unified Demo (Native)"
echo

# Check dependencies
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Install with: brew install python3"
    exit 1
fi

# Install dependencies if needed
pip3 install agent-ros-bridge websockets grpcio grpcio-tools paho-mqtt prometheus-client 2>/dev/null

# Generate JWT secret if not set
if [ -z "$JWT_SECRET" ]; then
    export JWT_SECRET=$(openssl rand -base64 32)
    echo "🔐 Generated JWT_SECRET"
fi

# Run the server
echo "🌐 Starting server on http://localhost:8080"
echo "📱 Open http://localhost:8080 in your browser"
echo "⚠️  Press Ctrl+C to stop"
echo

python3 demo_server.py
