# DevOps & TDD Implementation

This directory contains infrastructure as code and deployment configurations.

## Structure

```
infrastructure/
├── terraform/          # Infrastructure as Code
│   ├── main.tf
│   ├── variables.tf
│   └── outputs.tf
├── kubernetes/         # K8s manifests
│   ├── namespace.yaml
│   ├── deployment.yaml
│   ├── service.yaml
│   └── hpa.yaml
└── monitoring/         # Observability
    ├── prometheus.yaml
    └── grafana-dashboards/
```

## Quick Start

### 1. Provision Infrastructure

```bash
cd infrastructure/terraform
terraform init
terraform plan
terraform apply
```

### 2. Deploy to Kubernetes

```bash
kubectl apply -f infrastructure/kubernetes/
```

### 3. Verify Deployment

```bash
kubectl get pods -n agent-ros-bridge
kubectl logs -f deployment/agent-ros-bridge -n agent-ros-bridge
```

## Monitoring

Access Grafana:
```bash
kubectl port-forward svc/grafana 3000:3000 -n monitoring
# Open http://localhost:3000
```

## TDD Workflow

1. Write test in `tests/`
2. Run `pytest` (should fail)
3. Write code to pass test
4. Run `pytest` (should pass)
5. Refactor
6. Commit with tests

## CI/CD Pipeline

See `.github/workflows/devops.yml`

Stages:
1. Lint (Ruff, Black, MyPy)
2. Test (Unit, Integration with coverage)
3. Security (Trivy, Bandit)
4. Build (Docker multi-arch)
5. Deploy (Staging/Production)
