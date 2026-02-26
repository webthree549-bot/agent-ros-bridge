# Agent ROS Bridge — Production image
# This is a thin wrapper that delegates to docker/Dockerfile
# For CI compatibility: docker build -t agent-ros-bridge .

FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

COPY . .
RUN pip3 install --break-system-packages --no-cache-dir -e .

# Source ROS2 on bash login (non-interactive shells use ENV)
ENV ROS_DISTRO=jazzy
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENV PYTHONUNBUFFERED=1
EXPOSE 8765 50051

# Use the new module entry point
ENTRYPOINT ["python3", "-m", "agent_ros_bridge"]
CMD []
