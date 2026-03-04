# AI Framework Integration Examples

**Version:** v0.5.1  
**Purpose:** Demonstrate LangChain, AutoGPT, MCP, and Dashboard integrations

---

## Quick Start

```bash
# Set required secrets
export JWT_SECRET=$(openssl rand -base64 32)
export OPENAI_API_KEY=sk-...  # For LangChain example

# Run all examples
docker-compose up

# Or run specific example
docker-compose run --rm langchain-bridge python3 /app/langchain_example.py
```

---

## Examples

### 1. LangChain Integration (`langchain_example.py`)

**What it shows:** Using Agent ROS Bridge as a LangChain tool

**Prerequisites:**
- OpenAI API key
- langchain installed (in container)

**Run:**
```bash
export OPENAI_API_KEY=sk-...
docker-compose up langchain-bridge
```

### 2. AutoGPT Integration (`autogpt_example.py`)

**What it shows:** Agent ROS Bridge as AutoGPT command provider

**Run:**
```bash
docker-compose up autogpt-bridge
```

### 3. MCP Integration (`mcp_example.py`)

**What it shows:** Model Context Protocol server for Claude Desktop

**Run:**
```bash
docker-compose up mcp-bridge
```

### 4. Dashboard Integration (`dashboard_example.py`)

**What it shows:** Web dashboard for monitoring

**Run:**
```bash
docker-compose up dashboard-bridge
```

---

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│   AI Framework  │────▶│  Agent ROS Bridge│────▶│   Robot     │
│  (LangChain/etc)│     │  (WebSocket/MQTT)│     │  (ROS1/ROS2)│
└─────────────────┘     └──────────────────┘     └─────────────┘
```

---

## Troubleshooting

**"langchain not installed"**
- The Docker image includes all dependencies
- Run via docker-compose, not directly

**"OPENAI_API_KEY not set"**
- Required for LangChain example
- Get key from: https://platform.openai.com

---

*Part of Agent ROS Bridge v0.5.1*
