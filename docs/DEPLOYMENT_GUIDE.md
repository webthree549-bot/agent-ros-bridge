# Deployment Guide

Production deployment guide for Agent ROS Bridge.

## Deployment Options

| Environment | Method | Complexity | Best For |
|-------------|--------|------------|----------|
| Local Dev | Docker Compose | Low | Development |
| Single Server | Docker Compose | Low | Small fleets (<100) |
| Production | Kubernetes | Medium | Large fleets (1000+) |
| Enterprise | Kubernetes + Cloud | High | Mission-critical |

---

## Quick Start (Docker Compose)

### 1. Prerequisites

```bash
# Docker 24.0+
docker --version

# Docker Compose 2.20+
docker-compose --version

# 4+ CPU cores, 8GB RAM available
```

### 2. Configuration

Create `.env` file:

```bash
# Security (REQUIRED for production)
JWT_SECRET=$(openssl rand -base64 32)

# Database (optional - defaults to SQLite)
# DATABASE_URL=postgresql://user:pass@localhost/bridge

# Redis (optional - for large fleets)
# REDIS_URL=redis://localhost:6379/0

# Logging
LOG_LEVEL=INFO

# Ports
WEBSOCKET_PORT=8765
GRPC_PORT=50051
DASHBOARD_PORT=8081
```

### 3. Deploy

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Start services
docker-compose --profile web up -d

# Verify
docker-compose ps
curl http://localhost:8765/health
```

### 4. Access

- **Dashboard**: http://localhost:8081
- **API**: http://localhost:8765
- **WebSocket**: ws://localhost:8765

---

## Production Deployment (Kubernetes)

### 1. Prerequisites

```bash
# Kubernetes 1.28+
kubectl version --client

# Helm 3.13+
helm version

# cert-manager (for TLS)
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.0/cert-manager.yaml
```

### 2. Namespace

```bash
kubectl create namespace agent-ros-bridge
kubectl config set-context --current --namespace=agent-ros-bridge
```

### 3. Secrets

```bash
# Generate secrets
kubectl create secret generic bridge-secrets \
  --from-literal=JWT_SECRET=$(openssl rand -base64 32) \
  --from-literal=DATABASE_URL=$(echo -n 'postgresql://...' | base64)
```

### 4. Deploy with Helm

```bash
# Add Helm repo (when published)
helm repo add agent-ros-bridge https://charts.agent-ros-bridge.ai

# Install
helm install bridge agent-ros-bridge/bridge \
  --namespace agent-ros-bridge \
  --values values-production.yaml
```

Or use raw manifests:

```bash
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/secrets.yaml
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/deployment.yaml
kubectl apply -f k8s/service.yaml
kubectl apply -f k8s/ingress.yaml
```

### 5. Verify

```bash
# Check pods
kubectl get pods

# Check services
kubectl get svc

# Check ingress
kubectl get ingress

# View logs
kubectl logs -f deployment/bridge
```

---

## Cloud Deployment

### AWS

#### EKS (Recommended)

```bash
# Create EKS cluster
eksctl create cluster \
  --name agent-ros-bridge \
  --region us-west-2 \
  --node-type m6i.2xlarge \
  --nodes 3

# Deploy
kubectl apply -f k8s/
```

#### ECS (Alternative)

```bash
# Using AWS Copilot
copilot init --app bridge --svc api --dockerfile Dockerfile
copilot deploy
```

### Google Cloud

#### GKE

```bash
# Create cluster
gcloud container clusters create bridge-cluster \
  --zone us-central1-a \
  --machine-type n2-standard-4 \
  --num-nodes 3

# Deploy
kubectl apply -f k8s/
```

### Azure

#### AKS

```bash
# Create cluster
az aks create \
  --resource-group myResourceGroup \
  --name bridge-cluster \
  --node-count 3 \
  --generate-ssh-keys

# Deploy
kubectl apply -f k8s/
```

---

## High Availability

### Architecture

```
                    ┌─────────────────┐
                    │   Load Balancer │
                    │   (Cloudflare)  │
                    └────────┬────────┘
                             │
            ┌────────────────┼────────────────┐
            │                │                │
     ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
     │  Bridge-1   │  │  Bridge-2   │  │  Bridge-3   │
     │  (Primary)  │  │  (Replica)  │  │  (Replica)  │
     └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
            │                │                │
            └────────────────┼────────────────┘
                             │
                    ┌────────▼────────┐
                    │    Redis        │
                    │  (Session Store)│
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   PostgreSQL    │
                    │   (Primary)     │
                    └─────────────────┘
```

### Configuration

```yaml
# values-production.yaml
replicaCount: 3

resources:
  requests:
    cpu: 2000m
    memory: 4Gi
  limits:
    cpu: 4000m
    memory: 8Gi

autoscaling:
  enabled: true
  minReplicas: 3
  maxReplicas: 10
  targetCPUUtilizationPercentage: 70

redis:
  enabled: true
  replicaCount: 3

postgresql:
  enabled: true
  replication:
    enabled: true
    readReplicas: 2
```

---

## Security Hardening

### TLS/SSL

```bash
# Generate certificates
certbot certonly --standalone -d robots.example.com

# Create Kubernetes secret
kubectl create secret tls bridge-tls \
  --cert=/etc/letsencrypt/live/robots.example.com/fullchain.pem \
  --key=/etc/letsencrypt/live/robots.example.com/privkey.pem
```

### Network Policies

```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: bridge-network-policy
spec:
  podSelector:
    matchLabels:
      app: bridge
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - namespaceSelector:
        matchLabels:
          name: frontend
    ports:
    - protocol: TCP
      port: 8765
  egress:
  - to:
    - podSelector:
        matchLabels:
          app: postgres
    ports:
    - protocol: TCP
      port: 5432
```

### Pod Security

```yaml
securityContext:
  runAsNonRoot: true
  runAsUser: 1000
  fsGroup: 1000
  capabilities:
    drop:
    - ALL
  readOnlyRootFilesystem: true
  allowPrivilegeEscalation: false
```

---

## Monitoring

### Prometheus + Grafana

```bash
# Install kube-prometheus-stack
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm install monitoring prometheus-community/kube-prometheus-stack

# Access Grafana
kubectl port-forward svc/monitoring-grafana 3000:80
```

### Key Metrics

| Metric | Warning | Critical |
|--------|---------|----------|
| CPU Usage | >70% | >90% |
| Memory Usage | >80% | >95% |
| Latency P99 | >100ms | >500ms |
| Error Rate | >0.1% | >1% |
| Disk Usage | >80% | >95% |

### Alerts

```yaml
# alerts.yaml
groups:
- name: bridge-alerts
  rules:
  - alert: HighErrorRate
    expr: rate(bridge_errors_total[5m]) > 0.01
    for: 5m
    labels:
      severity: critical
    annotations:
      summary: "High error rate detected"
      
  - alert: RobotDisconnected
    expr: bridge_robots_connected < 1
    for: 1m
    labels:
      severity: warning
    annotations:
      summary: "All robots disconnected"
```

---

## Backup & Recovery

### Database Backup

```bash
# Automated backup (CronJob)
kubectl apply -f k8s/backup-cronjob.yaml

# Manual backup
kubectl exec -it postgres-0 -- pg_dump -U bridge bridge > backup.sql

# Restore
kubectl exec -i postgres-0 -- psql -U bridge bridge < backup.sql
```

### Configuration Backup

```bash
# Export all manifests
kubectl get all -o yaml > backup-manifests.yaml

# Export secrets (encrypted)
kubectl get secrets -o yaml > backup-secrets.yaml
```

---

## Scaling

### Vertical Scaling

```bash
# Increase CPU/memory
kubectl patch deployment bridge -p '{"spec":{"template":{"spec":{"containers":[{"name":"bridge","resources":{"requests":{"cpu":"4","memory":"8Gi"}}}]}}}}'
```

### Horizontal Scaling

```bash
# Manual scaling
kubectl scale deployment bridge --replicas=5

# Enable HPA
kubectl autoscale deployment bridge --min=3 --max=10 --cpu-percent=70
```

### Geographic Scaling

Deploy in multiple regions:
```
us-west-2: 3 replicas (primary)
us-east-1: 3 replicas (backup)
eu-west-1: 3 replicas (EU users)
```

---

## Troubleshooting

### Common Issues

#### Pods Not Starting
```bash
# Check events
kubectl describe pod bridge-xxx

# Check logs
kubectl logs bridge-xxx --previous
```

#### Connection Refused
```bash
# Check service
kubectl get svc bridge
kubectl get endpoints bridge

# Check network policy
kubectl get networkpolicies
```

#### High Memory Usage
```bash
# Check metrics
kubectl top pods

# Enable profiling
kubectl exec -it bridge-xxx -- python -m memory_profiler app.py
```

### Debug Commands

```bash
# Shell into pod
kubectl exec -it bridge-xxx -- /bin/sh

# Port forward for local debugging
kubectl port-forward pod/bridge-xxx 8765:8765

# View all resources
kubectl get all -o wide
```

---

## Maintenance

### Updates

```bash
# Rolling update
kubectl set image deployment/bridge bridge=agentrosbridge/bridge:v0.7.0

# Watch rollout
kubectl rollout status deployment/bridge

# Rollback if needed
kubectl rollout undo deployment/bridge
```

### Health Checks

```bash
# Daily check script
#!/bin/bash
echo "=== Bridge Health Check ==="
date

# Pod status
kubectl get pods

# Resource usage
kubectl top nodes
kubectl top pods

# Logs check
kubectl logs deployment/bridge --tail=100 | grep ERROR || echo "No errors"

# API health
curl -sf http://localhost:8765/health && echo "✅ Healthy" || echo "❌ Unhealthy"
```

---

## Production Checklist

- [ ] TLS certificates configured
- [ ] Secrets stored securely (not in Git)
- [ ] Network policies enabled
- [ ] Resource limits set
- [ ] Health checks configured
- [ ] Monitoring enabled
- [ ] Alerts configured
- [ ] Backups scheduled
- [ ] Disaster recovery tested
- [ ] Security audit passed
- [ ] Load testing completed
- [ ] Documentation updated
- [ ] Runbook created
- [ ] On-call rotation established

---

## Support

**Documentation**: https://docs.agent-ros-bridge.ai  
**Issues**: https://github.com/agent-ros-bridge/issues  
**Slack**: https://agent-ros-bridge.slack.com  
**Email**: support@agent-ros-bridge.ai

---

## License

MIT License - See [LICENSE](../LICENSE)