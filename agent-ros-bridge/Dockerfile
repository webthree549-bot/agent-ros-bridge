# Dockerfile for Agent ROS Bridge
# Multi-stage build for production deployment

# Stage 1: Base image with Python
FROM python:3.11-slim as base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Stage 2: Build dependencies
FROM base as builder

# Install build dependencies
RUN pip install --no-cache-dir build

# Copy source code
COPY . .

# Build wheel
RUN python -m build --wheel

# Stage 3: Production image
FROM base as production

# Create non-root user
RUN useradd -m -u 1000 bridge

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

# Copy wheel from builder
COPY --from=builder /app/dist/*.whl /tmp/

# Install package
RUN pip install --no-cache-dir /tmp/*.whl && rm /tmp/*.whl

# Create config directory
RUN mkdir -p /etc/agent-ros-bridge /var/log/agent-ros-bridge && \
    chown -R bridge:bridge /etc/agent-ros-bridge /var/log/agent-ros-bridge

# Switch to non-root user
USER bridge

# Expose ports
EXPOSE 8765 50051 8080 9090

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8080/health')" || exit 1

# Default command
CMD ["python", "-m", "agent_ros_bridge.mcp"]

# Stage 4: ROS2 development image
FROM ros:jazzy-ros-base as ros2

# Install Python
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy source
COPY . .

# Install package
RUN pip install --no-cache-dir -e ".[all]"

# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Expose ports
EXPOSE 8765 50051

# Default command
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && python -m agent_ros_bridge.mcp"]

# Stage 5: Development image
FROM base as dev

# Install development dependencies
RUN pip install --no-cache-dir -e ".[all,dev]"

# Copy source for development
COPY . .

# Install in editable mode
RUN pip install --no-cache-dir -e ".[all,dev]"

# Default command for development
CMD ["python", "-m", "agent_ros_bridge.mcp"]
