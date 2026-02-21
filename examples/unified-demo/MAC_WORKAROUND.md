# Docker Desktop Mac Workaround

## Problem
Docker Desktop for Mac has a port forwarding bug where `localhost:8080` doesn't connect to the container even though the container is running.

## Solution

Use the Mac-specific compose file with `network_mode: host`:

```bash
cd examples/unified-demo

# Use Mac-specific config
docker-compose -f docker-compose.mac.yml up -d

# Access via localhost:8080 (should work with host networking)
open http://localhost:8080
```

## Alternative Workarounds

### Option 1: Container IP Direct
```bash
# Get container's internal IP
CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' unified-demo)

# Access via IP
curl http://$CONTAINER_IP:8080/api/status
```

### Option 2: Port Forward via SSH Tunnel
```bash
# Forward container port to Mac localhost
docker exec unified-demo sh -c "apt-get update && apt-get install -y socat"
docker exec -d unified-demo socat TCP-LISTEN:8080,fork TCP:127.0.0.1:8080
```

### Option 3: Use Docker Desktop GUI
1. Open Docker Desktop
2. Go to Containers
3. Find unified-demo
4. Click "Open in Browser"

### Option 4: Shell Access
```bash
# Enter container and test from inside
docker exec -it unified-demo bash
curl http://localhost:8080/api/status
```

## Verify Server is Running

```bash
# Check server is responding inside container
docker exec unified-demo curl -s http://localhost:8080/api/status
```

If this returns JSON, the server is working fine - it's just the Mac→container networking that needs the workaround above.

## Known Issue

This is a documented Docker Desktop for Mac bug:
- https://github.com/docker/for-mac/issues/3611
- https://github.com/docker/compose/issues/9201

The `network_mode: host` workaround is the most reliable solution for Mac users.
