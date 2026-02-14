#!/usr/bin/env python3
"""Metrics Demo - Prometheus Monitoring

Demonstrates metrics collection and Prometheus export.

Usage:
    python demo/metrics_demo.py
    
Then view metrics:
    curl http://localhost:9090/metrics
"""

import asyncio
import logging
import random

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.metrics import MetricsServer, get_metrics

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("metrics_demo")


async def simulate_robot_activity(metrics):
    """Simulate robot activity for metrics demonstration"""
    robot_ids = [f"robot_{i:03d}" for i in range(5)]
    
    while True:
        # Simulate random robot connections
        active = random.randint(2, 5)
        metrics.set_robots_online(active)
        metrics.set_robots_total(5)
        
        # Simulate connections
        for _ in range(random.randint(1, 3)):
            metrics.record_connection_opened("websocket")
        
        metrics.set_active_connections(random.randint(3, 8), "websocket")
        
        # Simulate messages
        for _ in range(random.randint(5, 20)):
            metrics.record_message_sent("websocket", random.randint(100, 5000))
            await asyncio.sleep(0.01)
        
        for _ in range(random.randint(5, 20)):
            metrics.record_message_received("websocket", random.randint(100, 5000))
            await asyncio.sleep(0.01)
        
        # Simulate tasks
        for _ in range(random.randint(1, 3)):
            if random.random() > 0.1:  # 90% success
                metrics.record_task_completed(
                    task_type=random.choice(["navigate", "manipulate"]),
                    duration_sec=random.uniform(1.0, 10.0)
                )
            else:
                metrics.record_task_failed(task_type="navigate")
        
        metrics.set_task_queue_size(random.randint(0, 5))
        
        # Record response time
        metrics.record_response_time(random.uniform(0.001, 0.1))
        
        await asyncio.sleep(2)


async def main():
    # Create bridge
    bridge = Bridge()
    ws_transport = WebSocketTransport({'port': 8774})
    bridge.transport_manager.register(ws_transport)
    
    # Create and start metrics server
    metrics = get_metrics()
    server = MetricsServer(port=9090, collector=metrics)
    await server.start()
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print("📊 METRICS DEMO - Prometheus Monitoring")
    print("=" * 60)
    print("")
    print("Metrics endpoints:")
    print("  • Prometheus: http://localhost:9090/metrics")
    print("  • Health:     http://localhost:9090/health")
    print("")
    print("Try:")
    print("  curl http://localhost:9090/metrics")
    print("")
    print("For Grafana dashboard, import dashboards/grafana-dashboard.json")
    print("")
    print("Simulating robot activity...")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    # Start simulation
    simulator_task = asyncio.create_task(simulate_robot_activity(metrics))
    
    # Print metrics snapshot periodically
    async def print_snapshot():
        while True:
            await asyncio.sleep(10)
            snapshot = metrics.get_snapshot()
            print(f"\n📈 Snapshot: {snapshot.robots_online}/{snapshot.robots_total} robots, "
                  f"{snapshot.tasks_completed} tasks, "
                  f"{snapshot.messages_sent} msgs")
    
    snapshot_task = asyncio.create_task(print_snapshot())
    
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping...")
        simulator_task.cancel()
        snapshot_task.cancel()
        await bridge.stop()
        server.stop()


if __name__ == "__main__":
    asyncio.run(main())
