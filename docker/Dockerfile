# Agent ROS Bridge — Production image (ROS2 Jazzy)
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

ENTRYPOINT ["agent-ros-bridge"]
CMD []
