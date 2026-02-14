#!/bin/bash
# Metrics Example - Prometheus Monitoring

set -e

echo "📊 Starting Metrics Demo"
echo "========================="
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "metrics_demo.py" ]; then
    echo "❌ Run this script from the metrics/ directory"
    exit 1
fi

echo "Starting metrics server on http://localhost:9090"
echo ""
echo "View metrics:"
echo "  curl http://localhost:9090/metrics"
echo ""
echo "Grafana dashboard:"
echo "  Import ../../dashboards/grafana-dashboard.json"
echo "  Add Prometheus data source: http://localhost:9090"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 metrics_demo.py "$@"
