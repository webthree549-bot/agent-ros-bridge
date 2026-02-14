# Agent ROS Bridge with ROS2
FROM ros:jazzy-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    ros-jazzy-rosbridge-server \
    ros-jazzy-turtlebot4-gz \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages \
    websockets>=11.0 \
    grpcio>=1.56.0 \
    grpcio-tools>=1.56.0 \
    paho-mqtt>=1.6.0 \
    pyyaml>=6.0.1 \
    psutil>=5.9.5 \
    python-dotenv>=1.0.0

# Create workspace
WORKDIR /workspace

# Copy project
COPY . /workspace/

# Install the package
RUN pip3 install --break-system-packages -e .

# Source ROS2 on startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 run_bridge.py"]
