.PHONY: help install install-dev install-all test test-cov lint format clean docker-build docker-run docs serve-docs release check security

help:
	@echo "Agent ROS Bridge - Available Commands:"
	@echo ""
	@echo "Setup:"
	@echo "  make install        Install package"
	@echo "  make install-dev    Install with dev dependencies"
	@echo "  make install-all    Install with all extras"
	@echo ""
	@echo "Development:"
	@echo "  make test           Run tests"
	@echo "  make test-cov       Run tests with coverage"
	@echo "  make lint           Run linters"
	@echo "  make format         Auto-format code"
	@echo "  make check          Run all checks"
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
	@echo "  make release VERSION=x.x.x"

install:
	pip install -e .

install-dev:
	pip install -e ".[dev,test]"
	pre-commit install

install-all:
	pip install -e ".[all]"

test:
	pytest tests/ -v

test-cov:
	pytest tests/ -v --cov=agent_ros_bridge --cov-report=html --cov-report=term

lint:
	ruff check .
	black --check .
	isort --check-only .
	mypy agent_ros_bridge/ --ignore-missing-imports

format:
	black .
	isort .
	ruff check --fix .

check: lint test
	@echo "All checks passed!"

docs:
	mkdocs build

serve-docs:
	mkdocs serve

docker-build:
	docker build -t agent-ros-bridge:latest .

docker-run:
	docker run -p 8765:8765 agent-ros-bridge:latest

clean:
	rm -rf build/ dist/ *.egg-info/ .pytest_cache/ .mypy_cache/ htmlcov/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

release:
ifndef VERSION
	$(error VERSION is required. Usage: make release VERSION=x.x.x)
endif
	git tag -a "v$(VERSION)" -m "Release v$(VERSION)"
	git push origin "v$(VERSION)"

dev-server:
	agent-ros-bridge --demo
