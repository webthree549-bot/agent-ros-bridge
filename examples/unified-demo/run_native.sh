#!/bin/bash
# run_native.sh - Run unified demo directly on macOS without Docker

cd "$(dirname "$0")"

echo "🚀 Starting Agent ROS Bridge Unified Demo (Native)"
echo

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Install with: brew install python3"
    exit 1
fi

# Create virtual environment if not exists
if [ ! -d ".venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment
echo "🐍 Activating virtual environment..."
source .venv/bin/activate

# Install dependencies
echo "📦 Installing dependencies..."
pip install --upgrade pip -q
pip install agent-ros-bridge websockets grpcio grpcio-tools paho-mqtt prometheus-client aiohttp -q

# Generate JWT secret if not set
if [ -z "$JWT_SECRET" ]; then
    export JWT_SECRET=$(openssl rand -base64 32)
    echo "🔐 Generated JWT_SECRET"
    echo "   (Set your own with: export JWT_SECRET=your-secret)"
fi

# Run the server
echo "🌐 Starting server on http://localhost:8080"
echo "📱 Open http://localhost:8080 in your browser"
echo "⚠️  Press Ctrl+C to stop"
echo

python demo_server.py
