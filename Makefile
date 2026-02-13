.PHONY: help install install-dev install-all test test-cov lint format clean docker-build docker-run docs serve-docs release check security

# Default target
help:
	@echo "OpenClaw ROS Bridge - Available Commands:"
	@echo ""
	@echo "Setup:"
	@echo "  make install        Install package"
	@echo "  make install-dev    Install with dev dependencies"
	@echo "  make install-all    Install with all extras"
	@echo ""
	@echo "Development:"
	@echo "  make test           Run tests"
	@echo "  make test-cov       Run tests with coverage"
	@echo "  make lint           Run linters (ruff, black, isort, mypy)"
	@echo "  make format         Auto-format code"
	@echo "  make check          Run all checks (lint + test)"
	@echo "  make security       Run security checks"
	@echo ""
	@echo "Documentation:"
	@echo "  make docs           Build documentation"
	@echo "  make serve-docs     Serve docs locally"
	@echo ""
	@echo "Docker:"
	@echo "  make docker-build   Build Docker images"
	@echo "  make docker-run     Run Docker container"
	@echo ""
	@echo "Release:"
	@echo "  make release        Create a new release (requires VERSION)"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean          Clean build artifacts"
	@echo "  make update-deps    Update dependencies"

# Installation
install:
	pip install -e .

install-dev:
	pip install -e ".[dev,test]"
	pre-commit install

install-all:
	pip install -e ".[all]"

# Testing
test:
	pytest tests/ -v

test-cov:
	pytest tests/ -v --cov=openclaw_ros_bridge --cov-report=html --cov-report=term

test-integration:
	pytest tests/integration/ -v -m integration

test-ros:
	pytest tests/integration/ -v -m ros

# Linting and formatting
lint:
	ruff check .
	black --check .
	isort --check-only .
	mypy openclaw_ros_bridge/ --ignore-missing-imports

format:
	black .
	isort .
	ruff check --fix .

# Security checks
security:
	bandit -r openclaw_ros_bridge/ -f json -o bandit-report.json || true
	pip-audit --requirement requirements.txt || true

trivy:
	trivy fs . --scanners vuln,secret,config

# Combined checks
check: lint test security
	@echo "All checks passed!"

# Documentation
docs:
	mkdocs build

serve-docs:
	mkdocs serve

# Docker
docker-build:
	./scripts/docker_build.sh

docker-run:
	docker-compose -f docker/docker-compose.yml up -d

docker-stop:
	docker-compose -f docker/docker-compose.yml down

docker-logs:
	docker-compose -f docker/docker-compose.yml logs -f

# Release (requires VERSION=x.x.x)
release:
ifndef VERSION
	$(error VERSION is required. Usage: make release VERSION=x.x.x)
endif
	@echo "Creating release $(VERSION)..."
	git tag -a "v$(VERSION)" -m "Release v$(VERSION)"
	git push origin "v$(VERSION)"
	@echo "Release v$(VERSION) triggered. GitHub Actions will handle the rest."

# Changelog
changelog:
	git-chglog -o CHANGELOG.md

# Cleanup
clean:
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info/
	rm -rf .pytest_cache/
	rm -rf .mypy_cache/
	rm -rf htmlcov/
	rm -rf .coverage
	rm -rf site/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

# Dependency updates
update-deps:
	pip-compile --upgrade pyproject.toml
	pip-compile --upgrade --extra dev pyproject.toml -o requirements-dev.txt

# Development server
dev-server:
	python -m openclaw_ros_bridge.gateway_v2 --demo

# Pre-commit hooks
pre-commit:
	pre-commit run --all-files

# Benchmark
benchmark:
	pytest tests/benchmark/ --benchmark-only

# Quick test (fast unit tests only)
quick-test:
	pytest tests/unit/ -v -x

# CI simulation (run all CI checks locally)
ci: format lint test security
	@echo "Local CI checks complete!"
