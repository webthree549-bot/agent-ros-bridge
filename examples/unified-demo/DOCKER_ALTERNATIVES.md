# Docker Alternatives for Mac

## Option 1: Run Natively (Recommended)

**No Docker needed!** Run directly on macOS:

```bash
cd examples/unified-demo
./run_native.sh
```

Or manually:
```bash
cd examples/unified-demo
pip3 install agent-ros-bridge websockets grpcio grpcio-tools paho-mqtt prometheus-client
export JWT_SECRET=$(openssl rand -base64 32)
python3 demo_server.py
```

**Access:** http://localhost:8080 ✅ Works perfectly!

---

## Option 2: OrbStack (Docker Replacement)

**OrbStack** is a faster, lighter Docker Desktop alternative:

```bash
# Install OrbStack
brew install orbstack

# Use it exactly like Docker
cd examples/unified-demo
orbstack docker-compose up -d

# Access: http://localhost:8080
```

**Website:** https://orbstack.dev  
**Pros:** Faster, lighter, better Mac integration  
**Cons:** Still has some port forwarding quirks

---

## Option 3: Podman (Red Hat's Docker Alternative)

```bash
# Install Podman
brew install podman

# Initialize podman machine
podman machine init
podman machine start

# Run unified demo
cd examples/unified-demo
podman-compose up -d

# Access via podman machine IP
podman machine ip
# Then: http://<ip>:8080
```

---

## Option 4: Lima (Linux VM on Mac)

```bash
# Install Lima
brew install lima

# Start Ubuntu VM
limactl start template://docker

# Run in VM
lima docker-compose up -d

# Port forward from VM to Mac
limactl shell docker -- curl http://localhost:8080/api/status
```

---

## Option 5: GitHub Codespaces (Cloud)

```bash
# Open repo in GitHub Codespaces
# Everything works in cloud Linux environment
# Access via Codespaces port forwarding
```

---

## Quick Comparison

| Solution | Speed | Setup | Port Access | Recommendation |
|----------|-------|-------|-------------|----------------|
| **Native** | ⚡ Fastest | Easy | ✅ Perfect | ⭐ Best |
| **OrbStack** | Fast | Easy | ⚠️ Sometimes buggy | Good |
| **Podman** | Medium | Medium | ⚠️ Via VM IP | OK |
| **Lima** | Slow | Complex | ⚠️ Port forward | Advanced |
| **Codespaces** | Fast | None | ✅ Via GitHub | Cloud option |

---

## Recommended: Native

**Just run:**
```bash
cd examples/unified-demo
./run_native.sh
```

No Docker, no VMs, no port forwarding issues. Works immediately!
