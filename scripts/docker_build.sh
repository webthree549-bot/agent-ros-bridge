#!/bin/bash
# Docker build script
echo "Building Docker images..."
docker-compose -f docker/docker-compose.yml build