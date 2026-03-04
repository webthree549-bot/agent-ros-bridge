# Multi-stage Dockerfile for Agent ROS Bridge
# Supports: development, production, and testing

# =============================================================================
# Stage 1: Builder
# =============================================================================
FROM python:3.11-slim as builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
COPY requirements.txt requirements-dev.txt ./
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# =============================================================================
# Stage 2: Production
# =============================================================================
FROM python:3.11-slim as production

LABEL maintainer="Agent ROS Bridge Team <team@agentrosbridge.io>"
LABEL description="Universal interface for AI agents to control ROS robots"
LABEL version="0.5.0"

# Create non-root user
RUN groupadd -r bridge && useradd -r -g bridge bridge

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpq5 \
    curl \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Set working directory
WORKDIR /app

# Copy application code
COPY --chown=bridge:bridge agent_ros_bridge/ ./agent_ros_bridge/
COPY --chown=bridge:bridge skills/ ./skills/

# Create data directory
RUN mkdir -p /app/data && chown -R bridge:bridge /app/data

# Set environment variables
ENV PYTHONPATH=/app
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV BRIDGE_VERSION=0.5.0

# Switch to non-root user
USER bridge

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD curl -f http://localhost:8765/health || exit 1

# Expose ports
EXPOSE 8765 1883 50051

# Run application
ENTRYPOINT ["python", "-m", "agent_ros_bridge.cli"]
CMD ["start"]

# =============================================================================
# Stage 3: Development
# =============================================================================
FROM builder as development

RUN pip install --no-cache-dir -r requirements-dev.txt

WORKDIR /app

# Mount source code as volume for hot reload
VOLUME ["/app"]

EXPOSE 8765 1883 50051

CMD ["python", "-m", "agent_ros_bridge.cli", "start", "--log-level", "DEBUG"]

# =============================================================================
# Stage 4: Testing
# =============================================================================
FROM builder as testing

RUN pip install --no-cache-dir -r requirements-dev.txt

WORKDIR /app

COPY . .

CMD ["python", "-m", "pytest", "-v", "--cov=agent_ros_bridge"]
