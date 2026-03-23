# Deployment Guide

## Agent ROS Bridge v0.6.4

Complete deployment guide for production environments.

---

## Quick Start

### Installation

```bash
# From PyPI (recommended)
pip install agent-ros-bridge==0.6.4

# Or install with all extras
pip install agent-ros-bridge[all]==0.6.4

# Verify installation
python -c "from agent_ros_bridge import __version__; print(__version__)"
```

### Docker

```bash
# Pull image (when available)
docker pull ghcr.io/webthree549-bot/agent-ros-bridge:v0.6.4

# Run with ROS2
docker run -it --rm \
  --network host \
  ghcr.io/webthree549-bot/agent-ros-bridge:v0.6.4
```

---

## Production Deployment

### 1. Environment Setup

#### Requirements
- Python 3.11, 3.12, 3.13, or 3.14
- ROS2 Humble or Jazzy (for robot integration)
- Gazebo 11 or Gazebo Sim (for simulation)
- 4GB RAM minimum, 8GB recommended
- 10GB disk space

#### System Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  ros-humble-ros-base \
  gazebo11 \
  libgazebo11-dev

# macOS
brew install python@3.11
# Note: ROS2/Gazebo not fully supported on macOS
```

### 2. Virtual Environment

```bash
# Create virtual environment
python3 -m venv /opt/agent-ros-bridge/venv
source /opt/agent-ros-bridge/venv/bin/activate

# Install package
pip install agent-ros-bridge==0.6.4
```

### 3. Configuration

#### Environment Variables

```bash
# Required
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_PATH=/opt/agent-ros-bridge/models:$GAZEBO_MODEL_PATH

# Optional - LLM providers
export OPENAI_API_KEY="your-key-here"
export ANTHROPIC_API_KEY="your-key-here"
export MOONSHOT_API_KEY="your-key-here"

# Optional - Shadow mode
export SHADOW_DATA_DIR=/var/lib/agent-ros-bridge/shadow_data
export SHADOW_TARGET_HOURS=200

# Optional - Logging
export LOG_LEVEL=INFO
export LOG_FILE=/var/log/agent-ros-bridge/app.log
```

#### Configuration File

Create `/etc/agent-ros-bridge/config.yaml`:

```yaml
# Agent ROS Bridge Configuration

# Gateway settings
gateway:
  host: "0.0.0.0"
  port: 8080
  max_connections: 100

# Safety settings
safety:
  validator_enabled: true
  emergency_stop_timeout_ms: 50
  max_velocity: 1.0
  max_acceleration: 0.5

# Shadow mode settings
shadow_mode:
  enabled: true
  confidence_threshold: 0.7
  require_confirmation: true
  data_dir: "/var/lib/agent-ros-bridge/shadow_data"
  target_hours: 200

# Simulation settings
simulation:
  enabled: true
  num_worlds: 4
  headless: true
  timeout_seconds: 60

# LLM settings
llm:
  default_provider: "moonshot"
  providers:
    moonshot:
      model: "kimi-k2.5"
      temperature: 0.7
    openai:
      model: "gpt-4"
      temperature: 0.7

# Logging
logging:
  level: "INFO"
  format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
  handlers:
    - file
    - console
```

### 4. Systemd Service

Create `/etc/systemd/system/agent-ros-bridge.service`:

```ini
[Unit]
Description=Agent ROS Bridge
After=network.target ros2.target

[Service]
Type=simple
User=agent-ros-bridge
Group=agent-ros-bridge
WorkingDirectory=/opt/agent-ros-bridge
Environment=ROS_DOMAIN_ID=0
Environment=PYTHONPATH=/opt/agent-ros-bridge/venv/lib/python3.11/site-packages
Environment=PATH=/opt/agent-ros-bridge/venv/bin:/usr/local/bin:/usr/bin:/bin

ExecStart=/opt/agent-ros-bridge/venv/bin/python -m agent_ros_bridge.gateway
ExecReload=/bin/kill -HUP $MAINPID
KillMode=mixed
KillSignal=SIGTERM
TimeoutStopSec=30

Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable agent-ros-bridge
sudo systemctl start agent-ros-bridge
sudo systemctl status agent-ros-bridge
```

### 5. Shadow Mode Data Collection

Start collecting 200+ hours of AI vs human decision data:

```bash
# Create data directory
sudo mkdir -p /var/lib/agent-ros-bridge/shadow_data
sudo chown agent-ros-bridge:agent-ros-bridge /var/lib/agent-ros-bridge/shadow_data

# Start collection
sudo -u agent-ros-bridge /opt/agent-ros-bridge/venv/bin/python \
  /opt/agent-ros-bridge/venv/lib/python3.11/site-packages/agent_ros_bridge/shadow/collector.py

# Or use the deployment script
/opt/agent-ros-bridge/venv/bin/python scripts/deploy_shadow_collection.py
```

Monitor progress:

```bash
# View checkpoint
cat /var/lib/agent-ros-bridge/shadow_data/checkpoint.json

# View recent decisions
tail -f /var/lib/agent-ros-bridge/shadow_data/decisions_$(date +%Y-%m-%d).jsonl
```

### 6. Human Confirmation UI

Start the web interface:

```bash
# Start UI server
python -m agent_ros_bridge.ui.confirmation --port 8080

# Or in code
from agent_ros_bridge.ui.confirmation import ConfirmationUI

ui = ConfirmationUI(port=8080)
ui.start_server()
```

Access the UI at `http://localhost:8080`

### 7. Simulation (Optional)

Run 10K scenario validation:

```bash
# Generate and validate 10K scenarios
python -c "
from agent_ros_bridge.validation.scenario_10k import run_gate2_validation

result = run_gate2_validation(
    output_dir='/var/lib/agent-ros-bridge/validation',
    count=10000
)

print(f'Success rate: {result[\"success_rate\"]*100:.2f}%')
print(f'Gate 2 passed: {result[\"gate2_passed\"]}')
"
```

---

## Docker Compose Deployment

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  agent-ros-bridge:
    image: ghcr.io/webthree549-bot/agent-ros-bridge:v0.6.4
    container_name: agent-ros-bridge
    restart: unless-stopped
    
    environment:
      - ROS_DOMAIN_ID=0
      - SHADOW_DATA_DIR=/data/shadow
      - LOG_LEVEL=INFO
    
    volumes:
      - ./config:/etc/agent-ros-bridge:ro
      - shadow_data:/data/shadow
      - ./logs:/var/log/agent-ros-bridge
    
    ports:
      - "8080:8080"
    
    networks:
      - ros2-network
    
    # Required for ROS2
    ipc: host
    pid: host
    network_mode: host

  # Optional: Gazebo simulation
  gazebo:
    image: ghcr.io/webthree549-bot/agent-ros-bridge:ros2-humble
    container_name: gazebo
    restart: unless-stopped
    
    environment:
      - DISPLAY=:1
      - GAZEBO_MASTER_URI=http://localhost:11345
    
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    
    command: gz sim -s -r --headless-rendering
    
    network_mode: host

volumes:
  shadow_data:
    driver: local

networks:
  ros2-network:
    driver: bridge
```

Deploy:

```bash
docker-compose up -d
```

---

## Monitoring

### Health Checks

```bash
# Check service status
sudo systemctl status agent-ros-bridge

# Check logs
sudo journalctl -u agent-ros-bridge -f

# Check shadow data collection
python -c "
from agent_ros_bridge.shadow.collector import ShadowModeCollector
c = ShadowModeCollector(output_dir='/var/lib/agent-ros-bridge/shadow_data')
print(c.get_status())
"
```

### Metrics

Key metrics to monitor:

| Metric | Target | Alert Threshold |
|--------|--------|-----------------|
| Agreement Rate | >80% | <70% |
| Safety Violations | 0 | >0 |
| Response Time | <100ms | >500ms |
| CPU Usage | <70% | >90% |
| Memory Usage | <80% | >95% |

### Prometheus/Grafana (Optional)

Export metrics for monitoring:

```python
from agent_ros_bridge.shadow.hooks import ShadowModeHooks
from prometheus_client import start_http_server, Gauge

# Start metrics server
start_http_server(9090)

# Create gauges
agreement_rate = Gauge('shadow_agreement_rate', 'AI-human agreement rate')
total_decisions = Gauge('shadow_total_decisions', 'Total decisions logged')

# Update metrics
hooks = ShadowModeHooks()
stats = hooks.get_stats()
agreement_rate.set(stats['agreement_rate'])
total_decisions.set(stats['total_decisions'])
```

---

## Troubleshooting

### Common Issues

#### Issue: Cannot connect to ROS2

**Solution:**
```bash
# Check ROS2 environment
source /opt/ros/humble/setup.bash
echo $ROS_DOMAIN_ID

# Verify ROS2 is running
ros2 topic list
```

#### Issue: Gazebo not found

**Solution:**
```bash
# Install Gazebo
sudo apt-get install gazebo11 libgazebo11-dev

# Check installation
which gz
gz --version
```

#### Issue: Permission denied on shadow data

**Solution:**
```bash
# Fix permissions
sudo chown -R agent-ros-bridge:agent-ros-bridge /var/lib/agent-ros-bridge/shadow_data
sudo chmod 755 /var/lib/agent-ros-bridge/shadow_data
```

#### Issue: LLM API errors

**Solution:**
```bash
# Check API keys
export OPENAI_API_KEY="your-key"
export MOONSHOT_API_KEY="your-key"

# Test connection
python -c "
from agent_ros_bridge.ai.llm_parser import MoonshotParser
p = MoonshotParser(api_key='your-key')
print(p.test_connection())
"
```

---

## Security

### Best Practices

1. **Use environment variables** for secrets (API keys)
2. **Run as non-root user** (`agent-ros-bridge`)
3. **Enable safety validator** in production
4. **Regular backups** of shadow data
5. **Network isolation** for ROS2 topics

### Firewall Rules

```bash
# Allow only necessary ports
sudo ufw allow 8080/tcp  # Confirmation UI
sudo ufw allow 11345/tcp # Gazebo (if needed)
sudo ufw enable
```

---

## Backup and Recovery

### Backup Script

```bash
#!/bin/bash
# /opt/agent-ros-bridge/scripts/backup.sh

BACKUP_DIR=/backup/agent-ros-bridge/$(date +%Y%m%d)
mkdir -p $BACKUP_DIR

# Backup shadow data
tar czf $BACKUP_DIR/shadow_data.tar.gz /var/lib/agent-ros-bridge/shadow_data

# Backup config
cp /etc/agent-ros-bridge/config.yaml $BACKUP_DIR/

# Backup logs
tar czf $BACKUP_DIR/logs.tar.gz /var/log/agent-ros-bridge

echo "Backup complete: $BACKUP_DIR"
```

### Recovery

```bash
# Restore from backup
tar xzf backup/20260323/shadow_data.tar.gz -C /
cp backup/20260323/config.yaml /etc/agent-ros-bridge/
sudo systemctl restart agent-ros-bridge
```

---

## Support

- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation**: https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
- **Changelog**: https://github.com/webthree549-bot/agent-ros-bridge/blob/main/CHANGELOG.md

---

## License

MIT License - See LICENSE file for details.
