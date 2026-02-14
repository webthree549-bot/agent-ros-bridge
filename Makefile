# Agent ROS Bridge - Makefile
# Build automation for development and distribution

.PHONY: help install install-dev clean build test lint format docs dist upload check

# Default target
help:
	@echo "Agent ROS Bridge - Build Commands"
	@echo "================================="
	@echo ""
	@echo "Setup:"
	@echo "  make install       Install package from source"
	@echo "  make install-dev   Install with development dependencies"
	@echo ""
	@echo "Development:"
	@echo "  make clean         Remove all build artifacts"
	@echo "  make build         Build wheel and source distribution"
	@echo "  make test          Run all tests"
	@echo "  make test-unit     Run unit tests only"
	@echo "  make test-int      Run integration tests"
	@echo "  make lint          Run linters (ruff, black check)"
	@echo "  make format        Auto-format code (black, isort)"
	@echo ""
	@echo "Documentation:"
	@echo "  make docs          Build documentation"
	@echo "  make serve-docs    Serve docs locally"
	@echo ""
	@echo "Distribution:"
	@echo "  make dist          Build distribution packages"
	@echo "  make upload        Upload to PyPI (requires credentials)"
	@echo "  make upload-test   Upload to TestPyPI"
	@echo ""
	@echo "Validation:"
	@echo "  make check         Run all checks (lint, test, build)"
	@echo "  make validate      Validate ROS setup"
	@echo ""

# Installation
install:
	pip install -e .

install-dev:
	pip install -e ".[dev,transports]"
	pre-commit install 2>/dev/null || true

# Cleanup
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build/
	@rm -rf dist/
	@rm -rf *.egg-info/
	@rm -rf .eggs/
	@rm -rf .pytest_cache/
	@rm -rf .mypy_cache/
	@rm -rf .coverage
	@rm -rf htmlcov/
	@rm -rf .tox/
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@find . -type f -name "*.pyo" -delete 2>/dev/null || true
	@find . -type f -name "*.so" -delete 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@echo "Clean complete."

# Build
build: clean
	python -m build

dist: build

# Testing
test:
	pytest tests/ -v --tb=short

test-unit:
	pytest tests/unit/ -v --tb=short

test-int:
	pytest tests/integration/ -v --tb=short

test-openclaw:
	python tests/test_openclaw_integration.py

# Linting and formatting
lint:
	ruff check agent_ros_bridge/ tests/ demo/ scripts/
	black --check agent_ros_bridge/ tests/ demo/ scripts/

format:
	black agent_ros_bridge/ tests/ demo/ scripts/
	isort agent_ros_bridge/ tests/ demo/ scripts/

# Documentation
docs:
	@echo "Documentation is in docs/ directory"
	@echo "See docs/USER_MANUAL.md and docs/API_REFERENCE.md"

serve-docs:
	@echo "Starting documentation server..."
	@echo "Open http://localhost:8000"
	@python -m http.server 8000 --directory docs/ 2>/dev/null || \
		python -c "import http.server; import socketserver; import os; os.chdir('docs'); httpd = socketserver.TCPServer(('', 8000), http.server.SimpleHTTPRequestHandler); print('Serving at http://localhost:8000'); httpd.serve_forever()"

# Distribution
upload: dist
	python -m twine upload dist/*

upload-test: dist
	python -m twine upload --repository testpypi dist/*

# Validation
check: lint test
	@echo "All checks passed!"

validate:
	python scripts/validate_ros_setup.py

# Docker helpers
docker-build:
	docker-compose build

docker-up:
	docker-compose --profile ros2 up ros2-bridge

docker-down:
	docker-compose down

# Development server
run-mock:
	python demo/mock_bridge.py

run-bridge:
	python run_bridge.py

run-dashboard:
	python dashboard/server.py
