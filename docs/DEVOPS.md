# DevOps Pipeline for Agent ROS Bridge

## CI/CD Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Developer Push                                  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         1. Pre-commit Hooks                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Black     │  │   isort     │  │   flake8    │  │   mypy              │ │
│  │  Formatter  │  │   Import    │  │   Linter    │  │   Type Check        │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         2. Fast Feedback Loop                                │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                         Unit Tests (< 2 min)                            │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐ │ │
│  │  │   Core      │  │  Transports │  │   Safety    │  │   Utils        │ │ │
│  │  │   Tests     │  │   Tests     │  │   Tests     │  │   Tests        │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         3. Integration Tests                                 │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐ │ │
│  │  │   Docker    │  │   ROS2      │  │   LCM       │  │   Blueprint    │ │ │
│  │  │   Tests     │  │   Tests     │  │   Tests     │  │   Tests        │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         4. E2E & Performance                                 │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐ │ │
│  │  │    E2E      │  │   Load      │  │   Security  │  │   Chaos        │ │ │
│  │  │   Tests     │  │   Tests     │  │   Tests     │  │   Tests        │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         5. Build & Package                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Python    │  │   Docker    │  │   Helm      │  │   Documentation     │ │
│  │   Wheel     │  │   Images    │  │   Charts    │  │   Site              │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         6. Deploy to Staging                                 │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                    Kubernetes Staging Environment                       │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐ │ │
│  │  │   Bridge    │  │   Metrics   │  │   Logs      │  │   Traces       │ │ │
│  │  │   Pods      │  │   (Prom)    │  │   (ELK)     │  │   (Jaeger)     │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         7. Smoke Tests in Staging                            │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  Health Checks │ API Tests │ Integration Tests │ Performance Baseline  │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         8. Production Deploy                                 │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                    Kubernetes Production Environment                    │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐ │ │
│  │  │   Bridge    │  │   Metrics   │  │   Logs      │  │   Traces       │ │ │
│  │  │   Pods      │  │   (Prom)    │  │   (ELK)     │  │   (Jaeger)     │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         9. Continuous Monitoring                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Metrics   │  │   Alerts    │  │   SLO       │  │   Auto-rollback     │ │
│  │Collection   │  │   (PagerDuty│  │ Tracking    │  │   on Error          │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## GitHub Actions Workflow

```yaml
# .github/workflows/ci-cd.yml
name: CI/CD Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  # Stage 1: Code Quality (Fast feedback)
  code-quality:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      
      - name: Install dependencies
        run: |
          pip install black isort flake8 mypy
          pip install -e ".[dev]"
      
      - name: Black formatter
        run: black --check --diff agent_ros_bridge tests
      
      - name: isort
        run: isort --check-only --diff agent_ros_bridge tests
      
      - name: flake8
        run: flake8 agent_ros_bridge tests --max-line-length=100
      
      - name: mypy
        run: mypy agent_ros_bridge --ignore-missing-imports

  # Stage 2: Unit Tests
  unit-tests:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ['3.9', '3.10', '3.11', '3.12']
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python ${{ matrix.python-version }}
        uses: actions/setup-python@v5
        with:
          python-version: ${{ matrix.python-version }}
      
      - name: Install dependencies
        run: pip install -e ".[dev]"
      
      - name: Run unit tests
        run: pytest tests/unit -v --cov=agent_ros_bridge --cov-report=xml
      
      - name: Upload coverage
        uses: codecov/codecov-action@v3
        with:
          file: ./coverage.xml
          fail_ci_if_error: true

  # Stage 3: Integration Tests with Docker
  integration-tests:
    runs-on: ubuntu-latest
    needs: [code-quality, unit-tests]
    services:
      redis:
        image: redis:7-alpine
        ports:
          - 6379:6379
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Docker
        uses: docker/setup-buildx-action@v3
      
      - name: Build test image
        run: docker build -t agent-ros-bridge:test -f docker/Dockerfile.test .
      
      - name: Run integration tests
        run: |
          docker run --network host \
            -e REDIS_URL=redis://localhost:6379 \
            agent-ros-bridge:test \
            pytest tests/integration -v

  # Stage 4: E2E Tests
  e2e-tests:
    runs-on: ubuntu-latest
    needs: integration-tests
    steps:
      - uses: actions/checkout@v4
      
      - name: Start services
        run: docker-compose -f docker-compose.test.yml up -d
      
      - name: Run E2E tests
        run: |
          sleep 10  # Wait for services
          pytest tests/e2e -v --tb=short
      
      - name: Cleanup
        run: docker-compose -f docker-compose.test.yml down

  # Stage 5: Performance Tests
  performance-tests:
    runs-on: ubuntu-latest
    needs: integration-tests
    steps:
      - uses: actions/checkout@v4
      
      - name: Install dependencies
        run: pip install -e ".[dev]" locust
      
      - name: Run performance tests
        run: |
          pytest tests/performance -v
          locust -f tests/performance/locustfile.py --headless -u 100 -r 10 --run-time 1m

  # Stage 6: Security Scan
  security-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Run Bandit
        run: |
          pip install bandit
          bandit -r agent_ros_bridge -f json -o bandit-report.json || true
      
      - name: Run Safety
        run: |
          pip install safety
          safety check --json --output safety-report.json || true
      
      - name: Upload security reports
        uses: actions/upload-artifact@v4
        with:
          name: security-reports
          path: |
            bandit-report.json
            safety-report.json

  # Stage 7: Build and Push
  build-and-push:
    runs-on: ubuntu-latest
    needs: [e2e-tests, performance-tests, security-scan]
    if: github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Login to Docker Hub
        uses: docker/login-action@v3
        with:
          username: ${{ secrets.DOCKER_USERNAME }}
          password: ${{ secrets.DOCKER_PASSWORD }}
      
      - name: Build and push
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: |
            agentrosbridge/agent-ros-bridge:latest
            agentrosbridge/agent-ros-bridge:${{ github.sha }}
          cache-from: type=gha
          cache-to: type=gha,mode=max

  # Stage 8: Deploy to Staging
  deploy-staging:
    runs-on: ubuntu-latest
    needs: build-and-push
    environment: staging
    steps:
      - uses: actions/checkout@v4
      
      - name: Configure kubectl
        uses: azure/setup-kubectl@v3
      
      - name: Deploy to staging
        run: |
          kubectl config use-context staging
          helm upgrade --install agent-ros-bridge ./helm-chart \
            --namespace staging \
            --set image.tag=${{ github.sha }}
      
      - name: Smoke tests
        run: |
          sleep 30
          kubectl run smoke-test --rm -i --restart=Never \
            --image=curlimages/curl \
            -- curl http://agent-ros-bridge:8080/health

  # Stage 9: Deploy to Production
  deploy-production:
    runs-on: ubuntu-latest
    needs: deploy-staging
    environment: production
    if: github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4
      
      - name: Configure kubectl
        uses: azure/setup-kubectl@v3
      
      - name: Deploy to production
        run: |
          kubectl config use-context production
          helm upgrade --install agent-ros-bridge ./helm-chart \
            --namespace production \
            --set image.tag=${{ github.sha }} \
            --wait
      
      - name: Verify deployment
        run: |
          kubectl rollout status deployment/agent-ros-bridge -n production
```

## Pre-commit Configuration

```yaml
# .pre-commit-config.yaml
repos:
  # Code formatting
  - repo: https://github.com/psf/black
    rev: 24.1.1
    hooks:
      - id: black
        language_version: python3.11
        args: ['--line-length=100']

  # Import sorting
  - repo: https://github.com/pycqa/isort
    rev: 5.13.2
    hooks:
      - id: isort
        args: ['--profile=black', '--line-length=100']

  # Linting
  - repo: https://github.com/pycqa/flake8
    rev: 7.0.0
    hooks:
      - id: flake8
        args: ['--max-line-length=100', '--extend-ignore=E203,W503']

  # Type checking
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.8.0
    hooks:
      - id: mypy
        additional_dependencies: [types-all]
        args: ['--ignore-missing-imports']

  # Security
  - repo: https://github.com/PyCQA/bandit
    rev: 1.7.7
    hooks:
      - id: bandit
        args: ['-iii', '-ll']

  # Documentation
  - repo: https://github.com/pre-commit/pygrep-hooks
    rev: v1.10.0
    hooks:
      - id: rst-backticks
      - id: trailing-whitespace
      - id: end-of-file-fixer

  # Tests (quick)
  - repo: local
    hooks:
      - id: pytest-quick
        name: pytest-quick
        entry: pytest tests/unit -x -q
        language: system
        pass_filenames: false
        always_run: true
```

## Testing Strategy

### Test Pyramid

```
                    /\
                   /  \
                  / E2E \        <- 10% of tests
                 /  (33)  \       <- 5 min runtime
                /__________\
               /            \
              /  Integration  \   <- 20% of tests
             /    (100+)       \  <- 3 min runtime
            /____________________\
           /                      \
          /       Unit Tests        \ <- 70% of tests
         /          (587)            \ <- 30 sec runtime
        /______________________________\
```

### Test Categories

1. **Unit Tests** (587 tests, <30s)
   - Fast feedback
   - No external dependencies
   - 95% coverage target

2. **Integration Tests** (100+ tests, ~3min)
   - Docker-based
   - Redis/Database required
   - ROS2 in container

3. **E2E Tests** (33 tests, ~5min)
   - Full system
   - Real ROS2
   - Hardware optional

4. **Performance Tests**
   - Load testing with Locust
   - Latency benchmarks
   - Memory profiling

5. **Security Tests**
   - Bandit static analysis
   - Dependency scanning
   - Penetration tests

6. **Chaos Tests**
   - Network failures
   - Service degradation
   - Recovery validation

## Monitoring & Observability

### Metrics (Prometheus)

```python
# Key metrics to track
MESSAGES_SENT = Counter('bridge_messages_sent_total', 'Total messages sent')
MESSAGES_RECEIVED = Counter('bridge_messages_received_total', 'Total messages received')
ACTIVE_CONNECTIONS = Gauge('bridge_active_connections', 'Current connections')
LATENCY = Histogram('bridge_latency_seconds', 'Message latency')
ROBOTS_ONLINE = Gauge('bridge_robots_online', 'Robots currently connected')
```

### Logging (Structured)

```python
import structlog

logger = structlog.get_logger()

logger.info(
    "message_processed",
    message_id=msg_id,
    transport="websocket",
    latency_ms=5.2,
    robot_id="robot_001"
)
```

### Tracing (OpenTelemetry)

```python
from opentelemetry import trace

tracer = trace.get_tracer(__name__)

with tracer.start_as_current_span("process_message") as span:
    span.set_attribute("message.type", "command")
    span.set_attribute("robot.id", robot_id)
    # Process message
```

### Alerting Rules

```yaml
# alerting-rules.yml
groups:
  - name: agent-ros-bridge
    rules:
      - alert: HighErrorRate
        expr: rate(bridge_errors_total[5m]) > 0.1
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "High error rate detected"
      
      - alert: NoRobotsConnected
        expr: bridge_robots_online == 0
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "No robots connected"
      
      - alert: HighLatency
        expr: histogram_quantile(0.95, bridge_latency_seconds) > 0.1
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "95th percentile latency > 100ms"
```

## Infrastructure as Code

### Terraform

```hcl
# terraform/main.tf
module "agent_ros_bridge" {
  source = "./modules/agent-ros-bridge"
  
  cluster_name = "production"
  node_count   = 3
  instance_type = "c5.2xlarge"
  
  enable_monitoring = true
  enable_autoscaling = true
  
  domain = "bridge.agentros.io"
}
```

### Helm Chart

```yaml
# helm/values.yaml
replicaCount: 3

image:
  repository: agentrosbridge/agent-ros-bridge
  tag: latest
  pullPolicy: IfNotPresent

service:
  type: LoadBalancer
  ports:
    websocket: 8765
    lcm: 7667
    metrics: 9090

resources:
  limits:
    cpu: 2000m
    memory: 4Gi
  requests:
    cpu: 1000m
    memory: 2Gi

autoscaling:
  enabled: true
  minReplicas: 3
  maxReplicas: 10
  targetCPUUtilizationPercentage: 70

monitoring:
  enabled: true
  serviceMonitor:
    enabled: true
```

## Deployment Strategies

### Blue-Green Deployment

```bash
# Deploy green version
kubectl apply -f k8s/green/

# Wait for health
kubectl rollout status deployment/agent-ros-bridge-green

# Switch traffic
kubectl patch service agent-ros-bridge -p '{"spec":{"selector":{"version":"green"}}}'

# Monitor, rollback if needed
```

### Canary Deployment

```yaml
# Argo Rollout
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: agent-ros-bridge
spec:
  replicas: 5
  strategy:
    canary:
      steps:
        - setWeight: 20
        - pause: {duration: 10m}
        - setWeight: 50
        - pause: {duration: 10m}
        - setWeight: 100
```

## SLOs & Error Budgets

| SLO | Target | Measurement |
|-----|--------|-------------|
| Availability | 99.9% | Uptime over 30 days |
| Latency (p95) | <100ms | Message round-trip |
| Error Rate | <0.1% | 5-minute window |
| Recovery Time | <5min | From failure detection |

## Cost Optimization

- Spot instances for non-critical workloads
- Auto-scaling based on connection count
- Resource right-sizing via metrics
- CDN for static assets

## Security Hardening

- Pod Security Policies
- Network Policies
- Secrets management (Vault)
- Image scanning (Trivy)
- Runtime security (Falco)
