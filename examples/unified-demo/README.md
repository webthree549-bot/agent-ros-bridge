# Unified Demo - Quick Start

## Prerequisites

- Docker + Docker Compose
- JWT_SECRET environment variable (required for security)

## Generate JWT Secret

```bash
# Generate secure random secret
export JWT_SECRET=$(openssl rand -base64 32)

# Or create .env file
echo "JWT_SECRET=$(openssl rand -base64 32)" > .env
```

⚠️ **Security Note:** The demo will NOT start without JWT_SECRET set.

## Run the Demo

```bash
# Start unified demo
docker-compose up --build

# Open web UI
open http://localhost:8080
```

## Web UI

- Browse all 10 examples
- Click "▶ Start" to launch an example
- Click "📋 Logs" to view output
- Click "⏹ Stop" to stop an example

## Ports

| Port | Service |
|------|---------|
| 8080 | Web Dashboard |
| 8765 | WebSocket API |

## Stopping

```bash
# Stop all demos
docker-compose down
```
