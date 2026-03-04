# Agent ROS Bridge: Deployment Options Guide

## Quick Answer: All of the Above

Agent ROS Bridge supports **multiple deployment strategies** depending on your use case:

| Deployment | Best For | Complexity | Scale |
|------------|----------|------------|-------|
| **PyPI Package** | Python developers, prototyping | Low | Single instance |
| **Docker Hub** | Container users, CI/CD | Medium | Single/Multi instance |
| **Kubernetes** | Production, enterprise, scaling | High | 10-1000+ instances |
| **ClawHub** | OpenClaw users, skill distribution | Low | End-user focused |
| **APT/Homebrew** | System integration, power users | Medium | Single instance |

---

## 1. PyPI Package (Python Developers)

### When to Use
- You're a Python developer integrating into existing projects
- Prototyping and development
- Custom deployments
- CI/CD pipelines

### Installation
```bash
pip install agent-ros-bridge

# With all extras
pip install agent-ros-bridge[dev,test,docs]
```

### Usage
```python
from agent_ros_bridge import Bridge

bridge = Bridge()
adapter = bridge.get_openclaw_adapter()
```

### Pros ✅
- Easy Python integration
- Version management with pip
- Virtual environment support
- IDE autocomplete

### Cons ❌
- Requires Python knowledge
- Manual dependency management
- No auto-scaling
- Single instance only

### Best For
- **Development & prototyping**
- **Integration into Python projects**
- **CI/CD testing**

---

## 2. Docker Hub (Container Users)

### When to Use
- You want easy deployment without Python setup
- Running on edge devices
- CI/CD pipelines
- Multi-instance on single host

### Installation
```bash
# Pull image
docker pull agentrosbridge/agent-ros-bridge:latest

# Run
docker run -d \
  -p 8765:8765 \
  -e JWT_SECRET=$(openssl rand -base64 32) \
  agentrosbridge/agent-ros-bridge:latest
```

### Docker Compose
```yaml
version: '3.8'
services:
  bridge:
    image: agentrosbridge/agent-ros-bridge:latest
    ports:
      - "8765:8765"
      - "50051:50051"
    environment:
      - JWT_SECRET=${JWT_SECRET}
      - DATABASE_URL=${DATABASE_URL}
    volumes:
      - ./config:/app/config
      - bridge-data:/app/data
    restart: unless-stopped

  redis:
    image: redis:7-alpine
    volumes:
      - redis-data:/data

volumes:
  bridge-data:
  redis-data:
```

### Pros ✅
- No Python/environment setup
- Consistent across platforms
- Easy to run
- Version pinning
- Multi-arch support (amd64, arm64)

### Cons ❌
- Requires Docker knowledge
- Manual scaling
- No built-in high availability

### Best For
- **Quick deployment**
- **Edge devices**
- **Small to medium deployments**
- **CI/CD environments**

---

## 3. Kubernetes (Production/Enterprise)

### When to Use
- Production workloads
- High availability required
- Auto-scaling needed
- Multi-region deployment
- Enterprise security requirements

### Installation
```bash
# Add Helm repo
helm repo add agent-ros-bridge https://charts.agentrosbridge.io
helm repo update

# Install
helm install agent-ros-bridge agent-ros-bridge/agent-ros-bridge \
  --namespace agent-ros-bridge \
  --create-namespace \
  --set replicaCount=3 \
  --set autoscaling.enabled=true
```

### Helm Values
```yaml
# values-production.yaml
replicaCount: 5

image:
  repository: ghcr.io/webthree549-bot/agent-ros-bridge
  tag: v0.5.0

autoscaling:
  enabled: true
  minReplicas: 3
  maxReplicas: 20
  targetCPUUtilizationPercentage: 70

resources:
  requests:
    memory: "512Mi"
    cpu: "500m"
  limits:
    memory: "1Gi"
    cpu: "1000m"

ingress:
  enabled: true
  className: nginx
  hosts:
    - host: api.agentrosbridge.io
      paths:
        - path: /
          pathType: Prefix

tls:
  enabled: true
  certManager: true
```

### Pros ✅
- Auto-scaling
- High availability
- Rolling updates (zero downtime)
- Resource management
- Enterprise security
- Multi-cloud support

### Cons ❌
- High complexity
- Requires K8s expertise
- Infrastructure overhead
- Cost (cluster management)

### Best For
- **Production deployments**
- **Enterprise use**
- **High traffic loads**
- **Mission-critical systems**

---

## 4. ClawHub (OpenClaw Users)

### When to Use
- You're using OpenClaw as your AI agent platform
- Want natural language robot control
- Skill-based deployment
- End-user focused

### Installation
```bash
# Install via ClawHub
npx clawhub@latest install agent-ros-bridge

# Or manually download skill
wget https://clawhub.ai/skills/agent-ros-bridge/latest.skill
npx clawhub install ./agent-ros-bridge.skill
```

### Usage in OpenClaw
```
User: "Check the robots"
OpenClaw: [Loads agent-ros-bridge skill]
         [Connects to robot fleet]
         [Returns status]

User: "Move robot 1 to position A"
OpenClaw: [Interprets command]
         [Sends to Agent ROS Bridge]
         [Robot executes]
```

### Pros ✅
- Natural language interface
- No code required
- Skill versioning
- Community sharing
- Context persistence

### Cons ❌
- Requires OpenClaw platform
- Less customization
- Dependent on OpenClaw ecosystem

### Best For
- **OpenClaw users**
- **End users** (non-technical)
- **Natural language control**
- **Skill marketplace**

---

## 5. System Packages (APT/Homebrew)

### When to Use
- System-level integration
- Power users
- IoT/Edge devices with package managers
- Traditional sysadmin workflows

### APT (Ubuntu/Debian)
```bash
# Add repository
curl -s https://apt.agentrosbridge.io/gpg | sudo apt-key add -
echo "deb https://apt.agentrosbridge.io stable main" | sudo tee /etc/apt/sources.list.d/agent-ros-bridge.list

# Install
sudo apt update
sudo apt install agent-ros-bridge

# Start service
sudo systemctl start agent-ros-bridge
sudo systemctl enable agent-ros-bridge
```

### Homebrew (macOS)
```bash
# Tap
brew tap agentrosbridge/tap

# Install
brew install agent-ros-bridge

# Start service
brew services start agent-ros-bridge
```

### Pros ✅
- Native system integration
- Automatic updates
- Service management
- Familiar to sysadmins

### Cons ❌
- Platform-specific
- Limited to single instance
- Less flexible
- Slower release cycle

### Best For
- **System integration**
- **IoT devices**
- **Power users**
- **Traditional deployments**

---

## Deployment Decision Matrix

### Choose PyPI If:
- [x] You're a Python developer
- [x] Building custom integrations
- [x] Prototyping
- [x] Need programmatic access

### Choose Docker If:
- [x] Want easy deployment
- [x] Running on edge devices
- [x] Need consistent environment
- [x] Small to medium scale

### Choose Kubernetes If:
- [x] Production workload
- [x] Need high availability
- [x] Require auto-scaling
- [x] Enterprise security needs
- [x] Multi-region deployment

### Choose ClawHub If:
- [x] Using OpenClaw platform
- [x] Want natural language control
- [x] End-user focused
- [x] Skill marketplace benefits

### Choose System Packages If:
- [x] System-level integration
- [x] IoT/Edge deployment
- [x] Traditional sysadmin workflow
- [x] Automatic updates needed

---

## Recommended Deployment Strategy

### Development → Production Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Development │────▶│   Staging   │────▶│ Production  │
└─────────────┘     └─────────────┘     └─────────────┘
      │                    │                    │
   PyPI/local          Docker/K8s          K8s/Helm
   pip install -e     Single instance     HA cluster
```

### Multi-Environment Setup

| Environment | Deployment | Purpose |
|-------------|------------|---------|
| **Local Dev** | PyPI (`pip install -e .`) | Development, testing |
| **CI/CD** | Docker | Automated testing |
| **Staging** | Docker Compose | Pre-production validation |
| **Production** | Kubernetes | Live workload |
| **Edge** | Docker | Field deployment |
| **End Users** | ClawHub | Natural language access |

---

## Quick Start by Use Case

### "I want to try it out"
```bash
# Docker (fastest)
docker run -p 8765:8765 agentrosbridge/agent-ros-bridge
```

### "I'm a Python developer"
```bash
# PyPI
pip install agent-ros-bridge
python -c "from agent_ros_bridge import Bridge; b = Bridge()"
```

### "I need production deployment"
```bash
# Kubernetes
helm install agent-ros-bridge agent-ros-bridge/agent-ros-bridge
```

### "I use OpenClaw"
```bash
# ClawHub
npx clawhub install agent-ros-bridge
```

### "I want system integration"
```bash
# APT (Ubuntu/Debian)
sudo apt install agent-ros-bridge
```

---

## Summary

| Your Situation | Recommended Deployment |
|----------------|------------------------|
| Just trying it out | **Docker** |
| Python development | **PyPI** |
| Production/Enterprise | **Kubernetes** |
| OpenClaw user | **ClawHub** |
| System integration | **APT/Homebrew** |
| Edge/IoT | **Docker** |

**All deployment methods are officially supported.** Choose based on your infrastructure, expertise, and scale requirements.

---

**Need help choosing?** 
- Small scale (< 10 robots): Docker
- Medium scale (10-100 robots): Docker or K8s
- Large scale (100+ robots): Kubernetes
- End users: ClawHub
- Developers: PyPI
