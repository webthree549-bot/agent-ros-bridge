.PHONY: help install dev-install test test-unit test-integration test-e2e coverage lint format type-check security build docker-build docker-push deploy-staging deploy-prod clean docs

# Default target
help:
	@echo "Agent ROS Bridge - Development Commands"
	@echo ""
	@echo "Setup:"
	@echo "  make install        Install production dependencies"
	@echo "  make dev-install    Install development dependencies"
	@echo ""
	@echo "Testing:"
	@echo "  make test           Run all tests"
	@echo "  make test-unit      Run unit tests only"
	@echo "  make test-integration Run integration tests"
	@echo "  make test-e2e       Run E2E tests"
	@echo "  make coverage       Generate coverage report"
	@echo ""
	@echo "Code Quality:"
	@echo "  make lint           Run all linters"
	@echo "  make format         Format code with black and isort"
	@echo "  make type-check     Run mypy type checker"
	@echo "  make security       Run security scans"
	@echo ""
	@echo "Build:"
	@echo "  make build          Build Python package"
	@echo "  make docker-build   Build Docker image"
	@echo "  make docker-push    Push Docker image"
	@echo ""
	@echo "Deployment:"
	@echo "  make deploy-staging Deploy to staging"
	@echo "  make deploy-prod    Deploy to production"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean          Clean build artifacts"
	@echo "  make docs           Generate documentation"
	@echo "  make pre-commit     Install pre-commit hooks"

# Setup
install:
	pip install -e .

dev-install:
	pip install -e ".[dev]"
	pip install pre-commit black isort flake8 mypy bandit
	pre-commit install

# Testing
test: test-unit test-integration

test-unit:
	pytest tests/unit -v --tb=short --cov=agent_ros_bridge --cov-report=term-missing

test-integration:
	pytest tests/integration -v --tb=short

test-e2e:
	pytest tests/e2e -v --tb=short

test-quick:
	pytest tests/unit -x -q --tb=line

coverage:
	pytest tests/unit --cov=agent_ros_bridge --cov-report=html --cov-report=xml
	@echo "Coverage report generated in htmlcov/"

# Code Quality
lint: format-check type-check security

format:
	black agent_ros_bridge tests --line-length=100
	isort agent_ros_bridge tests --profile=black

format-check:
	black --check agent_ros_bridge tests --line-length=100
	isort --check-only agent_ros_bridge tests --profile=black
	flake8 agent_ros_bridge tests --max-line-length=100 --extend-ignore=E203,W503

type-check:
	mypy agent_ros_bridge --ignore-missing-imports

security:
	bandit -r agent_ros_bridge -ll
	safety check

# Build
build:
	python -m build

docker-build:
	docker build -t agent-ros-bridge:latest .

docker-push:
	docker tag agent-ros-bridge:latest ghcr.io/agentrosbridge/agent-ros-bridge:latest
	docker push ghcr.io/agentrosbridge/agent-ros-bridge:latest

# Deployment
deploy-staging:
	helm upgrade --install agent-ros-bridge ./helm-chart \
		--namespace staging \
		--values helm/values-staging.yaml

deploy-prod:
	helm upgrade --install agent-ros-bridge ./helm-chart \
		--namespace production \
		--values helm/values-production.yaml \
		--wait

# Maintenance
clean:
	rm -rf build/ dist/ *.egg-info/
	rm -rf htmlcov/ .coverage coverage.xml
	rm -rf .pytest_cache/ .mypy_cache/
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete

docs:
	cd docs && make html

pre-commit:
	pre-commit install
	pre-commit install --hook-type commit-msg

# Development shortcuts
run:
	python -m agent_ros_bridge

run-dev:
	LOG_LEVEL=DEBUG python -m agent_ros_bridge

docker-run:
	docker-compose up -d

docker-logs:
	docker-compose logs -f

docker-stop:
	docker-compose down

# CI/CD helpers
ci-test:
	make format-check
	make type-check
	make test-unit
	make security

ci-build:
	make build
	make docker-build

# Performance testing
perf-test:
	locust -f tests/performance/locustfile.py --headless -u 100 -r 10 --run-time 5m

# Monitoring
metrics:
	@echo "View metrics at http://localhost:9090"
	docker run -p 9090:9090 -v $(PWD)/monitoring/prometheus.yml:/etc/prometheus/prometheus.yml prom/prometheus

# Database migrations (if needed)
migrate:
	@echo "Running database migrations..."
	# alembic upgrade head

# Backup
backup:
	@echo "Creating backup..."
	# Add backup commands

# Restore
restore:
	@echo "Restoring from backup..."
	# Add restore commands
