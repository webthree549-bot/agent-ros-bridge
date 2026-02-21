FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install agent-ros-bridge with all extras
RUN pip3 install agent-ros-bridge[all] websockets grpcio grpcio-tools paho-mqtt prometheus-client

# Set up workspace
WORKDIR /app

# Copy demo server and web UI
COPY examples/unified-demo/demo_server.py /app/
COPY examples/unified-demo/static/index.html /app/static/index.html

# Copy all examples
COPY examples/ /app/examples/

# Environment
ENV PYTHONUNBUFFERED=1
ENV ROS_DOMAIN_ID=0

EXPOSE 8080 8765

CMD ["python3", "/app/demo_server.py"]
