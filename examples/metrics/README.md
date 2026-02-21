# Metrics Demo

Demonstrates Prometheus metrics collection with Agent ROS Bridge.

## Running

```bash
export JWT_SECRET=$(openssl rand -base64 32)
docker-compose up
```

## Metrics Endpoints

- Prometheus: http://localhost:9090/metrics
- Health: http://localhost:9090/health

## Metrics Collected

- Robot online/offline counts
- Message sent/received rates
- Task completion times
- Connection statistics
- Response times

## Grafana

Import `dashboards/grafana-dashboard.json` for visualization.
