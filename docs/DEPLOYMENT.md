# Agent ROS Bridge - Deployment Guide

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Running](#running)
5. [Docker Deployment](#docker-deployment)
6. [Production Setup](#production-setup)
7. [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4 cores | 8+ cores |
| RAM | 8 GB | 16+ GB |
| Storage | 20 GB SSD | 50+ GB SSD |
| Network | 100 Mbps | 1 Gbps |

### Software Requirements

- Python 3.9+
- ROS2 Humble (optional, for ROS integration)
- Docker 20.10+ (for containerized deployment)
- Git

### Operating Systems

- Ubuntu 22.04 LTS (recommended)
- macOS 13+ (development)
- Windows 11 + WSL2 (development)

## Installation

### Option 1: pip Installation

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install agent-ros-bridge
pip install agent-ros-bridge

# Install optional dependencies
pip install agent-ros-bridge[lcm,grpc]  # LCM and gRPC support
```

### Option 2: From Source

```bash
# Clone repository
git clone https://github.com/agent-ros-bridge/agent-ros-bridge.git
cd agent-ros-bridge

# Install dependencies
pip install -e ".[dev]"

# Run tests
pytest tests/
```

### Option 3: Docker

```bash
# Pull image
docker pull agentrosbridge/agent-ros-bridge:latest

# Or build from source
docker build -t agent-ros-bridge .
```

## Configuration

### Basic Configuration

Create `config/bridge.yaml`:

```yaml
bridge:
  name: "my_robot_bridge"
  ros_version: 2
  
  transports:
    websocket:
      enabled: true
      host: "0.0.0.0"
      port: 8765
      auth:
        enabled: true
        jwt_secret: ${JWT_SECRET}
    
    lcm:
      enabled: true
      udp_url: "udpm://239.255.76.67:7667"
      shared_memory: true
  
  safety:
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
    emergency_stop_timeout: 100  # ms
  
  logging:
    level: INFO
    format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
```

### Environment Variables

Create `.env`:

```bash
# Required
JWT_SECRET=your-secret-key-here

# Optional
ROS_VERSION=2
LOG_LEVEL=INFO
REDIS_URL=redis://localhost:6379
LCM_URL=udpm://239.255.76.67:7667

# Security
ENABLE_AUTH=true
ENABLE_TLS=false
TLS_CERT=/path/to/cert.pem
TLS_KEY=/path/to/key.pem
```

### ROS2 Integration

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Configure bridge for ROS2
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## Running

### Development Mode

```bash
# Start with default config
python -m agent_ros_bridge

# Start with custom config
python -m agent_ros_bridge --config config/bridge.yaml

# Start with specific transports
python -m agent_ros_bridge \
    --websocket-port 8765 \
    --lcm-url "udpm://239.255.76.67:7667"
```

### Production Mode

```bash
# Using production config
python -m agent_ros_bridge --config config/production.yaml

# With gunicorn (for REST API)
gunicorn -w 4 -k uvicorn.workers.UvicornWorker agent_ros_bridge.api:app
```

### Using Blueprints (New in v0.6.1)

```python
# robot_system.py
from agent_ros_bridge.gateway_v2 import Blueprint, autoconnect
from my_modules import CameraModule, DetectorModule, NavModule

async def main():
    # Create blueprint with autoconnect
    blueprint = autoconnect(
        CameraModule.blueprint(camera_id="front"),
        DetectorModule.blueprint(model="yolo"),
        NavModule.blueprint()
    )
    
    # Start system
    await blueprint.start()
    
    # Run forever
    await blueprint.loop()

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
```

## Docker Deployment

### Single Container

```bash
# Run bridge
docker run -d \
    --name agent-bridge \
    -p 8765:8765 \
    -p 7667:7667/udp \
    -v $(pwd)/config:/app/config \
    -e JWT_SECRET=${JWT_SECRET} \
    agentrosbridge/agent-ros-bridge:latest

# View logs
docker logs -f agent-bridge
```

### With ROS2

```yaml
# docker-compose.yml
version: '3.8'

services:
  ros2:
    image: osrf/ros:humble-desktop
    container_name: ros2
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    network_mode: host
    command: ros2 launch nav2_bringup navigation_launch.py

  bridge:
    image: agentrosbridge/agent-ros-bridge:latest
    container_name: bridge
    ports:
      - "8765:8765"
      - "7667:7667/udp"
    environment:
      - JWT_SECRET=${JWT_SECRET}
      - ROS_DOMAIN_ID=0
    volumes:
      - ./config:/app/config
    depends_on:
      - ros2
    command: --config /app/config/bridge.yaml

  dashboard:
    image: agentrosbridge/dashboard:latest
    container_name: dashboard
    ports:
      - "8080:80"
    environment:
      - BRIDGE_URL=ws://bridge:8765
```

```bash
# Start stack
docker-compose up -d

# View logs
docker-compose logs -f

# Stop
docker-compose down
```

### Kubernetes

```yaml
# k8s-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-ros-bridge
spec:
  replicas: 3
  selector:
    matchLabels:
      app: agent-ros-bridge
  template:
    metadata:
      labels:
        app: agent-ros-bridge
    spec:
      containers:
      - name: bridge
        image: agentrosbridge/agent-ros-bridge:latest
        ports:
        - containerPort: 8765
        - containerPort: 7667
          protocol: UDP
        env:
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: bridge-secrets
              key: jwt-secret
        volumeMounts:
        - name: config
          mountPath: /app/config
      volumes:
      - name: config
        configMap:
          name: bridge-config
---
apiVersion: v1
kind: Service
metadata:
  name: agent-ros-bridge
spec:
  selector:
    app: agent-ros-bridge
  ports:
  - name: websocket
    port: 8765
    targetPort: 8765
  - name: lcm
    port: 7667
    protocol: UDP
  type: LoadBalancer
```

## Production Setup

### Security Checklist

- [ ] Change default JWT secret
- [ ] Enable TLS/SSL
- [ ] Configure firewall rules
- [ ] Set up rate limiting
- [ ] Enable audit logging
- [ ] Configure CORS policies
- [ ] Set up monitoring

### Performance Tuning

```yaml
# config/production.yaml
bridge:
  transports:
    websocket:
      port: 8765
      max_connections: 10000
      heartbeat_interval: 30
      
    lcm:
      udp_url: "udpm://239.255.76.67:7667"
      shared_memory: true
      queue_size: 10000
  
  performance:
    worker_processes: 4
    thread_pool_size: 100
    max_message_size: 10MB
    
  safety:
    validation_timeout: 10  # ms
    emergency_stop_latency: 1  # ms
```

### Monitoring

```python
# monitoring.py
from agent_ros_bridge.metrics import get_metrics

async def monitor():
    metrics = get_metrics()
    
    while True:
        snapshot = metrics.get_snapshot()
        
        # Log metrics
        print(f"Robots online: {snapshot.robots_online}")
        print(f"Messages/sec: {snapshot.messages_sent}")
        print(f"Active connections: {snapshot.active_connections}")
        
        # Alert on issues
        if snapshot.robots_online == 0:
            send_alert("No robots connected!")
        
        await asyncio.sleep(60)
```

### Backup and Recovery

```bash
# Backup configuration
tar -czf backup-$(date +%Y%m%d).tar.gz config/ data/

# Backup Redis (if used)
redis-cli BGSAVE
cp /var/lib/redis/dump.rdb backup/

# Restore
tar -xzf backup-20240101.tar.gz
docker-compose restart
```

## Troubleshooting

### Common Issues

#### 1. Bridge won't start

```bash
# Check logs
python -m agent_ros_bridge 2>&1 | tee bridge.log

# Verify config
python -c "import yaml; yaml.safe_load(open('config/bridge.yaml'))"

# Check ports
netstat -tlnp | grep 8765
lsof -i :8765
```

#### 2. ROS2 connection failed

```bash
# Verify ROS2 installation
ros2 topic list

# Check domain ID
echo $ROS_DOMAIN_ID

# Test connectivity
ros2 topic pub /test std_msgs/String "data: 'hello'"
ros2 topic echo /test
```

#### 3. High latency

```bash
# Check network
ping robot-ip

# Monitor CPU/memory
htop

# Profile Python
cProfile -o profile.stats -m agent_ros_bridge
python -m pstats profile.stats
```

#### 4. LCM transport not working

```bash
# Check multicast
ifconfig | grep MULTICAST

# Test LCM
python -c "
from agent_ros_bridge.gateway_v2.transports import LCMTransport
import asyncio

t = LCMTransport({})
asyncio.run(t.start())
print('LCM started')
"
```

### Debug Mode

```bash
# Enable debug logging
LOG_LEVEL=DEBUG python -m agent_ros_bridge

# With profiling
python -m cProfile -s cumulative -m agent_ros_bridge
```

### Getting Help

1. Check logs: `logs/bridge.log`
2. Review documentation: https://docs.agent-ros-bridge.ai
3. GitHub Issues: https://github.com/agent-ros-bridge/issues
4. Discord: https://discord.gg/agent-ros-bridge

## Advanced Topics

### Custom Transports

```python
from agent_ros_bridge.gateway_v2.core import Transport

class CustomTransport(Transport):
    name = "custom"
    
    async def start(self) -> bool:
        # Initialize transport
        return True
    
    async def stop(self) -> None:
        # Cleanup
        pass
    
    async def send(self, message, recipient: str) -> bool:
        # Send message
        return True
    
    async def broadcast(self, message) -> list:
        # Broadcast to all
        return []
```

### Custom Modules

```python
from agent_ros_bridge.gateway_v2 import Module, In, Out, skill

class MyModule(Module):
    input: In[dict]
    output: Out[dict]
    
    @skill
    def do_something(self, param: str) -> bool:
        """AI-callable skill."""
        return True
    
    async def run(self):
        while self._running:
            data = await self.input.get()
            result = self.process(data)
            await self.output.publish(result)
```

### Health Checks

```python
from agent_ros_bridge.metrics import HealthChecker

checker = HealthChecker(bridge)

# Register custom checks
checker.register_check("ros2", lambda: ros2_node.is_alive())
checker.register_check("database", lambda: db.is_connected())

# Check health
health = await checker.check_health()
print(health["status"])  # "healthy" or "unhealthy"
```
