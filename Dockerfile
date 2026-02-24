# Agent ROS Bridge — Development / CI image (no ROS, pure Python)
# For ROS-aware images use docker/Dockerfile.ros1 or docker/Dockerfile.ros2
FROM python:3.11-slim

WORKDIR /app

# Install system deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies first (cache layer)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Install the package in editable mode
COPY . .
RUN pip install --no-cache-dir -e .

ENV PYTHONUNBUFFERED=1

EXPOSE 8765 50051

ENTRYPOINT ["agent-ros-bridge"]
CMD ["--help"]
