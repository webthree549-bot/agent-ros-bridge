#!/bin/bash
# MQTT IoT Example - Sensor Integration

set -e

echo "📡 Starting MQTT IoT Demo"
echo "=========================="
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "mqtt_demo.py" ]; then
    echo "❌ Run this script from the mqtt_iot/ directory"
    exit 1
fi

echo "Starting MQTT bridge on ws://localhost:8770"
echo ""
echo "This demo shows MQTT sensor integration."
echo ""
echo "MQTT Broker: localhost:1883 (optional)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 mqtt_demo.py "$@"
