# Multi-stage Dockerfile for production
FROM python:3.11-slim as builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
COPY requirements.txt requirements-dev.txt ./
RUN pip install --no-cache-dir -r requirements.txt -r requirements-dev.txt

# Production stage
FROM python:3.11-slim as production

# Create non-root user
RUN groupadd -r bridge && useradd -r -g bridge bridge

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpq5 \
    && rm -rf /var/lib/apt/lists/*

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Set working directory
WORKDIR /app

# Copy application code
COPY --chown=bridge:bridge agent_ros_bridge/ ./agent_ros_bridge/
COPY --chown=bridge:bridge skills/ ./skills/

# Set environment variables
ENV PYTHONPATH=/app
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Switch to non-root user
USER bridge

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8765/health')" || exit 1

# Expose ports
EXPOSE 8765 50051

# Run application
CMD ["python", "-m", "agent_ros_bridge.gateway_v2.core"]

# Development stage
FROM builder as development

RUN pip install --no-cache-dir -r requirements-dev.txt

WORKDIR /app

# Mount source code as volume for hot reload
VOLUME ["/app"]

CMD ["python", "-m", "pytest", "-v", "--watch"]
