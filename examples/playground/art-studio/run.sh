#!/bin/bash
# Quick run script for Art Studio with ROS2 + HTTP

cd ~/dev/workspace/playground/art-studio

# Set JWT
export JWT_SECRET=$(openssl rand -base64 32)

# Fix agent_ros_bridge type hints if needed
if [ -d "agent_ros_bridge" ]; then
    sed -i '' 's/async def _handle_client(self, websocket: WebSocketServerProtocol):/async def _handle_client(self, websocket):/' agent_ros_bridge/gateway_v2/transports/websocket.py 2>/dev/null || true
    sed -i '' 's/self.clients: Dict\[str, WebSocketServerProtocol\]/self.clients: Dict[str, Any]/' agent_ros_bridge/gateway_v2/transports/websocket.py 2>/dev/null || true
fi

# Run with volume mount to use local fixed code
docker run -d \
    --name art-studio \
    -p 8765:8765 \
    -p 8080:8080 \
    -e JWT_SECRET=$JWT_SECRET \
    -e ROS_DOMAIN_ID=0 \
    -v $(pwd)/agent_ros_bridge:/usr/local/lib/python3.10/dist-packages/agent_ros_bridge \
    -v $(pwd)/art_brain_ros2_http.py:/ros2_ws/art_brain_ros2_http.py \
    -v $(pwd)/canvas.html:/ros2_ws/canvas.html \
    ros:humble-ros-base \
    /bin/bash -c "
        apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/* &&
        pip3 install websockets==11.0.3 aiohttp>=3.8 numpy &&
        pip3 install /usr/local/lib/python3.10/dist-packages/agent_ros_bridge/ &&
        source /opt/ros/humble/setup.bash &&
        python3 /ros2_ws/art_brain_ros2_http.py
    "

echo "Art Studio starting..."
echo "WebSocket: ws://localhost:8765"
echo "HTTP: http://localhost:8080"
echo ""
echo "Wait 5 seconds then check logs:"
echo "docker logs art-studio"