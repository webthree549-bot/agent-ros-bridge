# v0.5.0 Integration Examples

This directory contains examples demonstrating the v0.5.0 AI agent integrations.

## Examples

### 1. LangChain Integration

```bash
python langchain_example.py
```

Shows how to use ROSBridgeTool with LangChain agents.

**Requirements:**
```bash
pip install langchain openai
```

### 2. AutoGPT Integration

```bash
python autogpt_example.py
```

Shows how to expose bridge actions as AutoGPT commands.

### 3. MCP Server (Claude Desktop)

```bash
python mcp_example.py
```

Starts an MCP server for Claude Desktop integration.

**Claude Desktop Config:**
Add to your Claude Desktop configuration file to enable robot control.

### 4. Dashboard

```bash
python dashboard_example.py
```

Starts the web dashboard on http://localhost:8080

## Common Setup

All examples require:

1. **JWT Secret:**
   ```bash
   export JWT_SECRET=$(openssl rand -base64 32)
   ```

2. **Installed Package:**
   ```bash
   pip install agent-ros-bridge
   ```

3. **ROS Environment** (for real robots):
   ```bash
   source /opt/ros/humble/setup.bash  # or jazzy
   ```

## Safety Features

All examples include:
- ✅ JWT authentication
- ✅ Safety confirmation for dangerous actions
- ✅ Emergency stop capability
- ✅ Audit logging

## Next Steps

- Try the examples with simulated robots (Docker)
- Integrate with your AI agent
- Build custom tools using the Bridge API
