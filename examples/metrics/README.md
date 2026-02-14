# Metrics Example

**Prometheus monitoring and Grafana dashboards.**

## What It Does

Exposes robot metrics for Prometheus scraping with a pre-built Grafana dashboard.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh
```

## View Metrics

```bash
# Raw metrics
curl http://localhost:9090/metrics

# Specific metrics
curl http://localhost:9090/metrics | grep agent_ros_bridge_robots_online
curl http://localhost:9090/metrics | grep agent_ros_bridge_tasks_completed
```

## Grafana Setup

1. Start Grafana: `docker run -p 3000:3000 grafana/grafana`
2. Open http://localhost:3000 (admin/admin)
3. Add Prometheus data source: http://host.docker.internal:9090
4. Import dashboard: `../../dashboards/grafana-dashboard.json`

## What's Happening

This demonstrates:
- **Prometheus Metrics**: Standard exposition format
- **Robot Telemetry**: Online count, battery, location
- **Task Metrics**: Completion rate, duration
- **System Health**: Message throughput, latency

## Key Metrics

| Metric | Description |
|--------|-------------|
| `robots_online` | Currently connected robots |
| `tasks_completed_total` | Cumulative task count |
| `task_duration_seconds` | Task execution time |
| `messages_sent_total` | Bridge message throughput |

## Next Steps

- Import dashboard in production Grafana
- Set up alerting rules
- Read [User Manual - Monitoring](../../docs/USER_MANUAL.md#monitoring)
