#!/bin/bash
# Start Shadow Mode Data Collection
# Launches bridge, dashboard, and monitoring

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  🤖 Shadow Mode Data Collection                           ║"
echo "║  Target: 200 hours with >95% agreement                   ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Check Docker
if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker."
    exit 1
fi

if ! docker info &> /dev/null; then
    log_error "Docker daemon not running. Please start Docker."
    exit 1
fi

# Start services
log_info "Starting bridge and dashboard..."
docker-compose --profile web up -d

# Wait for services
log_info "Waiting for services to start..."
sleep 5

# Check health
if curl -s http://localhost:8765/health > /dev/null; then
    log_success "Bridge is healthy"
else
    log_warn "Bridge health check failed, but continuing..."
fi

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  🚀 Services Started                                      ║"
echo "╠═══════════════════════════════════════════════════════════╣"
echo "║  Dashboard:   http://localhost:8081                      ║"
echo "║  Bridge API:  http://localhost:8765                      ║"
echo "║  WebSocket:   ws://localhost:8765                        ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Start monitor in background
log_info "Starting shadow mode monitor..."
if command -v python3 &> /dev/null; then
    if [ -f "scripts/shadow_monitor.py" ]; then
        python3 scripts/shadow_monitor.py &
        MONITOR_PID=$!
        log_success "Monitor started (PID: $MONITOR_PID)"
    else
        log_warn "Monitor script not found"
    fi
else
    log_warn "Python3 not found, skipping monitor"
fi

echo ""
echo "💡 Next Steps:"
echo "   1. Open dashboard: http://localhost:8081"
echo "   2. Click 'Connect' to link to bridge"
echo "   3. Operate robots and approve/reject AI proposals"
echo "   4. Watch progress in Shadow Mode section"
echo ""
echo "📊 Monitor Progress:"
echo "   - Dashboard: Shadow Mode section"
echo "   - Terminal: Monitor is running"
echo "   - Log file: shadow_mode_progress.log"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Wait for interrupt
trap 'cleanup' INT
cleanup() {
    echo ""
    log_info "Shutting down..."
    docker-compose down
    if [ ! -z "$MONITOR_PID" ]; then
        kill $MONITOR_PID 2>/dev/null || true
    fi
    log_success "Shutdown complete"
    exit 0
}

# Keep script running
while true; do
    sleep 1
done