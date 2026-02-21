#!/bin/bash
# Simple run script - builds and runs art studio

cd "$(dirname "$0")"

export JWT_SECRET=${JWT_SECRET:-$(openssl rand -base64 32)}

echo "Building Art Studio Docker image..."
docker build -f Dockerfile.ros2 -t art-studio-local .

echo ""
echo "Running Art Studio..."
docker run -d \
    --name art-studio \
    -p 8765:8765 \
    -p 8080:8080 \
    -e JWT_SECRET=$JWT_SECRET \
    art-studio-local

echo ""
echo "Services starting..."
echo "  HTTP: http://localhost:8080"
echo "  WebSocket: ws://localhost:8765"
echo ""
echo "Check logs: docker logs -f art-studio"
echo "Stop: docker stop art-studio && docker rm art-studio"