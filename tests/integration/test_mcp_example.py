"""Integration tests for MCP example.

TDD: Tests for examples/v0.5.0_integrations/mcp_example.py
"""

import pytest
from pathlib import Path
import sys

# Add examples to path
examples_dir = Path(__file__).parent.parent.parent / "examples" / "v0.5.0_integrations"
sys.path.insert(0, str(examples_dir))


class TestMCPExample:
    """Test MCP integration example."""

    def test_example_imports(self):
        """Test that mcp_example.py can be imported."""
        try:
            import mcp_example
            assert True
        except ImportError:
            pytest.skip("mcp_example dependencies not available")

    def test_example_file_exists(self):
        """Test that example file exists."""
        example_file = examples_dir / "mcp_example.py"
        assert example_file.exists()

    def test_example_structure(self):
        """Test that example has required structure."""
        example_file = examples_dir / "mcp_example.py"
        content = example_file.read_text()
        
        # Should have main function
        assert "async def main()" in content or "def main()" in content
        
        # Should use Bridge
        assert "Bridge" in content

    def test_docker_compose_exists(self):
        """Test that Docker Compose file exists."""
        compose_file = examples_dir / "docker-compose.yml"
        assert compose_file.exists()

    def test_readme_exists(self):
        """Test that README exists."""
        readme_file = examples_dir / "README.md"
        assert readme_file.exists()
