# DevOps & TDD Implementation Plan

## Executive Summary

Transform Agent ROS Bridge into a production-grade system with enterprise-level DevOps practices and strict TDD methodology.

---

## Phase 1: Infrastructure as Code (IaC)

### 1.1 Terraform Configuration

```hcl
# infrastructure/main.tf
terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.0"
    }
  }
}

# EKS Cluster for Agent ROS Bridge
module "eks" {
  source  = "terraform-aws-modules/eks/aws"
  version = "~> 19.0"

  cluster_name    = "agent-ros-bridge"
  cluster_version = "1.28"

  vpc_id     = module.vpc.vpc_id
  subnet_ids = module.vpc.private_subnets

  eks_managed_node_groups = {
    main = {
      desired_size = 3
      min_size     = 2
      max_size     = 10

      instance_types = ["m6i.large"]
      capacity_type  = "ON_DEMAND"
    }
  }
}

# RDS for Context Database
resource "aws_db_instance" "context" {
  identifier     = "agent-ros-bridge-context"
  engine         = "postgres"
  engine_version = "15.0"
  instance_class = "db.t3.medium"

  allocated_storage = 100
  storage_encrypted = true

  db_name  = "context_db"
  username = "admin"
  password = var.db_password

  backup_retention_period = 7
  multi_az               = true
}

# ElastiCache for Redis
resource "aws_elasticache_cluster" "redis" {
  cluster_id           = "agent-ros-bridge-cache"
  engine               = "redis"
  node_type            = "cache.t3.micro"
  num_cache_nodes      = 2
  parameter_group_name = "default.redis7"
  port                 = 6379
}
```

### 1.2 Kubernetes Manifests

```yaml
# k8s/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: agent-ros-bridge
  labels:
    app: agent-ros-bridge
    version: v0.5.0

---
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-ros-bridge
  namespace: agent-ros-bridge
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  selector:
    matchLabels:
      app: agent-ros-bridge
  template:
    metadata:
      labels:
        app: agent-ros-bridge
    spec:
      containers:
      - name: bridge
        image: agent-ros-bridge:v0.5.0
        ports:
        - containerPort: 8765
          name: websocket
        - containerPort: 50051
          name: grpc
        env:
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: bridge-secrets
              key: jwt-secret
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: bridge-secrets
              key: database-url
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8765
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8765
          initialDelaySeconds: 5
          periodSeconds: 5

---
# k8s/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: agent-ros-bridge
  namespace: agent-ros-bridge
spec:
  type: LoadBalancer
  ports:
  - port: 8765
    targetPort: 8765
    name: websocket
  - port: 50051
    targetPort: 50051
    name: grpc
  selector:
    app: agent-ros-bridge

---
# k8s/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: agent-ros-bridge
  namespace: agent-ros-bridge
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: agent-ros-bridge
  minReplicas: 3
  maxReplicas: 20
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

### 1.3 Docker Configuration

```dockerfile
# Dockerfile.multi-stage
# Stage 1: Builder
FROM python:3.11-slim as builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    postgresql-client \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt

# Stage 2: Production
FROM python:3.11-slim as production

# Create non-root user
RUN groupadd -r bridge && useradd -r -g bridge bridge

WORKDIR /app

# Copy only necessary files from builder
COPY --from=builder /root/.local /home/bridge/.local
COPY --chown=bridge:bridge agent_ros_bridge/ ./agent_ros_bridge/
COPY --chown=bridge:bridge skills/ ./skills/

# Set environment
ENV PATH=/home/bridge/.local/bin:$PATH
ENV PYTHONPATH=/app

# Switch to non-root user
USER bridge

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import agent_ros_bridge; print('OK')" || exit 1

EXPOSE 8765 50051

CMD ["python", "-m", "agent_ros_bridge.gateway_v2.core"]
```

---

## Phase 2: CI/CD Pipeline (GitHub Actions)

### 2.1 Main CI Pipeline

```yaml
# .github/workflows/ci-cd.yml
name: CI/CD Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

env:
  PYTHON_VERSION: "3.11"
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}

jobs:
  # Stage 1: Code Quality
  lint:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: ${{ env.PYTHON_VERSION }}
      
      - name: Install dependencies
        run: |
          pip install ruff black mypy bandit
      
      - name: Run Ruff (linting)
        run: ruff check agent_ros_bridge/ tests/
      
      - name: Run Black (formatting check)
        run: black --check agent_ros_bridge/ tests/
      
      - name: Run MyPy (type checking)
        run: mypy agent_ros_bridge/ --ignore-missing-imports
      
      - name: Run Bandit (security)
        run: bandit -r agent_ros_bridge/ -f json -o bandit-report.json || true
      
      - name: Upload security report
        uses: actions/upload-artifact@v4
        with:
          name: security-report
          path: bandit-report.json

  # Stage 2: Testing (TDD)
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        test-type: [unit, integration, e2e]
    services:
      postgres:
        image: postgres:15
        env:
          POSTGRES_PASSWORD: postgres
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
        ports:
          - 5432:5432
      redis:
        image: redis:7
        ports:
          - 6379:6379
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: ${{ env.PYTHON_VERSION }}
      
      - name: Install dependencies
        run: |
          pip install -e ".[dev,test]"
      
      - name: Run ${{ matrix.test-type }} tests
        run: |
          if [ "${{ matrix.test-type }}" == "unit" ]; then
            pytest tests/unit -v --cov=agent_ros_bridge --cov-report=xml
          elif [ "${{ matrix.test-type }}" == "integration" ]; then
            pytest tests/integration -v --cov=agent_ros_bridge --cov-report=xml --cov-append
          else
            pytest tests/e2e -v
          fi
        env:
          DATABASE_URL: postgresql://postgres:postgres@localhost:5432/test
          REDIS_URL: redis://localhost:6379/0
          JWT_SECRET: test-secret-for-ci
      
      - name: Upload coverage
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml
          flags: ${{ matrix.test-type }}
          name: ${{ matrix.test-type }}-coverage

  # Stage 3: Security Scanning
  security:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Run Trivy vulnerability scanner
        uses: aquasecurity/trivy-action@master
        with:
          scan-type: 'fs'
          scan-ref: '.'
          format: 'sarif'
          output: 'trivy-results.sarif'
      
      - name: Upload Trivy scan results
        uses: github/codeql-action/upload-sarif@v2
        with:
          sarif_file: 'trivy-results.sarif'
      
      - name: Run Snyk to check for vulnerabilities
        uses: snyk/actions/python@master
        env:
          SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}
        with:
          args: --severity-threshold=high

  # Stage 4: Build & Push
  build:
    needs: [lint, test, security]
    runs-on: ubuntu-latest
    permissions:
      contents: read
      packages: write
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Log in to Container Registry
        uses: docker/login-action@v3
        with:
          registry: ${{ env.REGISTRY }}
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      
      - name: Extract metadata
        id: meta
        uses: docker/metadata-action@v5
        with:
          images: ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}
          tags: |
            type=ref,event=branch
            type=ref,event=pr
            type=semver,pattern={{version}}
            type=semver,pattern={{major}}.{{minor}}
      
      - name: Build and push
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: ${{ steps.meta.outputs.tags }}
          labels: ${{ steps.meta.outputs.labels }}
          cache-from: type=gha
          cache-to: type=gha,mode=max
          platforms: linux/amd64,linux/arm64

  # Stage 5: Deploy to Staging
  deploy-staging:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/develop'
    environment: staging
    steps:
      - uses: actions/checkout@v4
      
      - name: Configure AWS credentials
        uses: aws-actions/configure-aws-credentials@v4
        with:
          aws-access-key-id: ${{ secrets.AWS_ACCESS_KEY_ID }}
          aws-secret-access-key: ${{ secrets.AWS_SECRET_ACCESS_KEY }}
          aws-region: us-west-2
      
      - name: Update kubeconfig
        run: aws eks update-kubeconfig --name agent-ros-bridge-staging
      
      - name: Deploy to staging
        run: |
          kubectl set image deployment/agent-ros-bridge \
            bridge=${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:develop \
            -n agent-ros-bridge
          kubectl rollout status deployment/agent-ros-bridge -n agent-ros-bridge
      
      - name: Run smoke tests
        run: |
          kubectl run smoke-test --rm -i --restart=Never \
            --image=curlimages/curl \
            -- curl -f http://agent-ros-bridge.agent-ros-bridge.svc.cluster.local:8765/health

  # Stage 6: Deploy to Production
  deploy-production:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/main'
    environment: production
    steps:
      - uses: actions/checkout@v4
      
      - name: Configure AWS credentials
        uses: aws-actions/configure-aws-credentials@v4
        with:
          aws-access-key-id: ${{ secrets.AWS_ACCESS_KEY_ID }}
          aws-secret-access-key: ${{ secrets.AWS_SECRET_ACCESS_KEY }}
          aws-region: us-west-2
      
      - name: Update kubeconfig
        run: aws eks update-kubeconfig --name agent-ros-bridge-prod
      
      - name: Deploy to production (canary)
        run: |
          # Canary deployment: 10% traffic first
          kubectl apply -f k8s/canary/
          kubectl set image deployment/agent-ros-bridge-canary \
            bridge=${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:main \
            -n agent-ros-bridge
          
          # Wait for canary health check
          sleep 60
          
          # If healthy, roll out to 100%
          kubectl set image deployment/agent-ros-bridge \
            bridge=${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:main \
            -n agent-ros-bridge
          kubectl rollout status deployment/agent-ros-bridge -n agent-ros-bridge
```

---

## Phase 3: TDD Implementation Workflow

### 3.1 TDD Cycle

```
┌─────────────────────────────────────────────────────────────┐
│                    TDD CYCLE                                │
│                                                              │
│   1. Write Test → 2. Run Test (Fail) → 3. Write Code       │
│        ↑                                    │                │
│        └──────── 5. Refactor ← 4. Run Test (Pass)           │
│                                                              │
│   Requirements:                                              │
│   - Test coverage > 95%                                      │
│   - All tests pass before merge                              │
│   - No code without tests                                    │
│   - Mutation testing for quality                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Example: TDD for New Feature

```python
# tests/test_new_feature.py
# Step 1: Write test FIRST (before implementation)

import pytest
from agent_ros_bridge.integrations.new_feature import NewFeature

class TestNewFeature:
    """TDD for new feature implementation."""
    
    @pytest.fixture
    def feature(self):
        return NewFeature()
    
    def test_feature_initialization(self, feature):
        """Test that feature can be initialized."""
        assert feature is not None
        assert feature.status == "ready"
    
    def test_feature_execute_success(self, feature):
        """Test successful execution."""
        # Given
        input_data = {"command": "test"}
        
        # When
        result = feature.execute(input_data)
        
        # Then
        assert result["success"] is True
        assert "output" in result
    
    def test_feature_execute_failure(self, feature):
        """Test failure handling."""
        # Given
        invalid_input = {"command": None}
        
        # When
        result = feature.execute(invalid_input)
        
        # Then
        assert result["success"] is False
        assert "error" in result
    
    def test_feature_edge_cases(self, feature):
        """Test edge cases."""
        # Empty input
        result = feature.execute({})
        assert result["success"] is False
        
        # Large input
        large_input = {"command": "x" * 10000}
        result = feature.execute(large_input)
        assert result["success"] is True


# agent_ros_bridge/integrations/new_feature.py
# Step 3: Write minimal code to pass tests

class NewFeature:
    """New feature implementation."""
    
    def __init__(self):
        self.status = "ready"
    
    def execute(self, input_data: dict) -> dict:
        """Execute feature."""
        # Step 3: Minimal implementation
        if not input_data or "command" not in input_data:
            return {"success": False, "error": "Invalid input"}
        
        if input_data["command"] is None:
            return {"success": False, "error": "Command is None"}
        
        # Step 5: Refactored implementation
        try:
            output = self._process(input_data["command"])
            return {"success": True, "output": output}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _process(self, command: str) -> str:
        """Process command (extracted for testability)."""
        return f"Processed: {command[:100]}"  # Truncate for safety
```

### 3.3 Test Categories

```yaml
# Test pyramid
tests/
  unit/                    # 70% of tests - Fast, isolated
    test_module_a.py
    test_module_b.py
    
  integration/             # 20% of tests - Component interactions
    test_database.py
    test_api.py
    test_robot_connection.py
    
  e2e/                     # 10% of tests - Full workflows
    test_greenhouse_scenario.py
    test_warehouse_workflow.py
    
  performance/             # Load testing
    test_load.py
    test_stress.py
    
  security/                # Security testing
    test_injection.py
    test_authentication.py
    test_authorization.py
    
  mutation/                # Mutation testing
    # Uses mutmut to check test quality
```

---

## Phase 4: Monitoring & Observability

### 4.1 Prometheus Metrics

```python
# agent_ros_bridge/monitoring/metrics.py
from prometheus_client import Counter, Histogram, Gauge, Info

# Request metrics
nl_requests_total = Counter(
    'nl_requests_total',
    'Total natural language requests',
    ['status', 'intent']
)

nl_request_duration = Histogram(
    'nl_request_duration_seconds',
    'NL request processing time',
    buckets=[0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0]
)

# Robot metrics
robot_connections = Gauge(
    'robot_connections_active',
    'Number of active robot connections',
    ['robot_type']
)

robot_command_duration = Histogram(
    'robot_command_duration_seconds',
    'Robot command execution time',
    ['command_type']
)

# System metrics
context_db_size = Gauge(
    'context_db_size_bytes',
    'Size of context database'
)

memory_usage = Gauge(
    'memory_usage_bytes',
    'Memory usage',
    ['type']  # rss, vms
)

# Business metrics
skills_executed = Counter(
    'skills_executed_total',
    'Total skills executed',
    ['skill_name', 'status']
)

user_sessions = Gauge(
    'user_sessions_active',
    'Number of active user sessions'
)
```

### 4.2 Grafana Dashboards

```json
{
  "dashboard": {
    "title": "Agent ROS Bridge - Production",
    "panels": [
      {
        "title": "Request Rate",
        "targets": [
          {
            "expr": "rate(nl_requests_total[5m])",
            "legendFormat": "{{status}}"
          }
        ]
      },
      {
        "title": "Response Time",
        "targets": [
          {
            "expr": "histogram_quantile(0.95, rate(nl_request_duration_bucket[5m]))",
            "legendFormat": "95th percentile"
          }
        ]
      },
      {
        "title": "Active Robots",
        "targets": [
          {
            "expr": "robot_connections",
            "legendFormat": "{{robot_type}}"
          }
        ]
      },
      {
        "title": "Error Rate",
        "targets": [
          {
            "expr": "rate(nl_requests_total{status=\"error\"}[5m])",
            "legendFormat": "Errors/sec"
          }
        ]
      }
    ]
  }
}
```

### 4.3 Logging with Structured JSON

```python
# agent_ros_bridge/logging_config.py
import logging
import json
from pythonjsonlogger import jsonlogger

class CustomJsonFormatter(jsonlogger.JsonFormatter):
    def add_fields(self, log_record, record, message_dict):
        super().add_fields(log_record, record, message_dict)
        log_record["service"] = "agent-ros-bridge"
        log_record["version"] = "0.5.0"

# Configure logging
logHandler = logging.StreamHandler()
formatter = CustomJsonFormatter(
    '%(timestamp)s %(level)s %(name)s %(message)s'
)
logHandler.setFormatter(formatter)

logger = logging.getLogger()
logger.addHandler(logHandler)
logger.setLevel(logging.INFO)

# Usage
logger.info("Processing NL command", extra={
    "command": "Move forward",
    "session_id": "abc123",
    "user_id": "user456",
    "processing_time_ms": 45
})
```

### 4.4 Distributed Tracing

```python
# agent_ros_bridge/tracing.py
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

# Configure tracing
trace.set_tracer_provider(TracerProvider())
tracer = trace.get_tracer(__name__)

otlp_exporter = OTLPSpanExporter(endpoint="otel-collector:4317")
span_processor = BatchSpanProcessor(otlp_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)

# Usage
@tracer.start_as_current_span("nl_interpretation")
def interpret_nl(command: str):
    with tracer.start_as_current_span("parameter_extraction"):
        params = extract_params(command)
    
    with tracer.start_as_current_span("intent_classification"):
        intent = classify_intent(command)
    
    return {"intent": intent, "params": params}
```

---

## Phase 5: Security & Compliance

### 5.1 Security Scanning in CI

```yaml
# .github/workflows/security.yml
name: Security Scan

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]
  schedule:
    - cron: '0 0 * * 0'  # Weekly

jobs:
  security-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Run Bandit
        run: |
          pip install bandit
          bandit -r agent_ros_bridge/ -f json -o bandit-report.json
      
      - name: Run Safety
        run: |
          pip install safety
          safety check --json --output safety-report.json
      
      - name: Run Trivy
        uses: aquasecurity/trivy-action@master
        with:
          scan-type: 'fs'
          format: 'sarif'
          output: 'trivy-results.sarif'
      
      - name: Run Snyk
        uses: snyk/actions/python@master
        env:
          SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}
      
      - name: Upload results
        uses: github/codeql-action/upload-sarif@v2
        with:
          sarif_file: 'trivy-results.sarif'
```

### 5.2 Secrets Management

```yaml
# External Secrets Operator
apiVersion: external-secrets.io/v1beta1
kind: ExternalSecret
metadata:
  name: bridge-secrets
  namespace: agent-ros-bridge
spec:
  refreshInterval: 1h
  secretStoreRef:
    kind: ClusterSecretStore
    name: aws-secrets-manager
  target:
    name: bridge-secrets
    creationPolicy: Owner
  data:
    - secretKey: jwt-secret
      remoteRef:
        key: agent-ros-bridge/prod
        property: jwt-secret
    - secretKey: database-url
      remoteRef:
        key: agent-ros-bridge/prod
        property: database-url
```

---

## Phase 6: Documentation as Code

### 6.1 Automated Documentation

```yaml
# .github/workflows/docs.yml
name: Documentation

on:
  push:
    branches: [main]
    paths:
      - 'docs/**'
      - 'agent_ros_bridge/**/*.py'

jobs:
  docs:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Generate API docs
        run: |
          pip install pdoc3
          pdoc3 --html --output-dir docs/api agent_ros_bridge
      
      - name: Build MkDocs
        run: |
          pip install mkdocs mkdocs-material
          mkdocs build
      
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./site
```

---

## Implementation Timeline

| Phase | Duration | Key Deliverables |
|-------|----------|------------------|
| Phase 1 | Week 1-2 | Terraform, K8s, Docker |
| Phase 2 | Week 3-4 | CI/CD pipeline |
| Phase 3 | Week 5-6 | TDD workflow, 95% coverage |
| Phase 4 | Week 7-8 | Monitoring, logging, tracing |
| Phase 5 | Week 9-10 | Security scanning, compliance |
| Phase 6 | Week 11-12 | Documentation automation |

**Total: 12 weeks to production-grade DevOps**

---

## Success Criteria

### DevOps Maturity
- [ ] Infrastructure as Code (100%)
- [ ] Automated CI/CD (< 10 min build)
- [ ] Zero-downtime deployments
- [ ] Auto-scaling configured
- [ ] Disaster recovery tested

### TDD Maturity
- [ ] 95%+ test coverage
- [ ] < 100ms test execution
- [ ] Mutation score > 80%
- [ ] No code without tests
- [ ] Property-based testing

### Observability
- [ ] 99.9% uptime monitoring
- [ ] < 1 min alert response
- [ ] Full distributed tracing
- [ ] Structured logging
- [ ] Custom dashboards

### Security
- [ ] Zero critical vulnerabilities
- [ ] Automated security scanning
- [ ] Secrets rotation
- [ ] Compliance (SOC2, ISO)
- [ ] Penetration testing

---

**Next Steps:**
1. Set up AWS account and Terraform backend
2. Create EKS cluster
3. Implement CI/CD pipeline
4. Start TDD for new features
5. Configure monitoring stack

**Status:** Ready for implementation
