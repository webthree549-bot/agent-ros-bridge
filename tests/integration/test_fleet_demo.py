"""Integration tests for Fleet demo.

TDD: Tests for examples/fleet/fleet_demo.py
"""

import pytest
import asyncio
from pathlib import Path
import sys

# Add examples to path
examples_dir = Path(__file__).parent.parent.parent / "examples" / "fleet"
sys.path.insert(0, str(examples_dir))


class TestFleetDemo:
    """Test fleet orchestration example."""

    def test_example_imports(self):
        """Test that fleet_demo.py can be imported."""
        try:
            import fleet_demo
            assert True
        except ImportError:
            pytest.skip("fleet_demo dependencies not available")

    def test_example_file_exists(self):
        """Test that example file exists."""
        example_file = examples_dir / "fleet_demo.py"
        assert example_file.exists()

    def test_example_structure(self):
        """Test that example has required structure."""
        example_file = examples_dir / "fleet_demo.py"
        content = example_file.read_text()
        
        # Should have main function
        assert "async def main()" in content or "def main()" in content
        
        # Should use FleetOrchestrator or Bridge
        assert "FleetOrchestrator" in content or "Bridge" in content

    def test_docker_compose_exists(self):
        """Test that Docker Compose file exists."""
        compose_file = examples_dir / "docker-compose.yml"
        assert compose_file.exists()

    def test_readme_exists(self):
        """Test that README exists."""
        readme_file = examples_dir / "README.md"
        assert readme_file.exists()
