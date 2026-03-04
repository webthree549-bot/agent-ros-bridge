"""Integration tests for LangChain example.

TDD: Tests for examples/v0.5.0_integrations/langchain_example.py
"""

import pytest
import sys
from pathlib import Path

# Add examples to path
examples_dir = Path(__file__).parent.parent.parent / "examples" / "v0.5.0_integrations"
sys.path.insert(0, str(examples_dir))


class TestLangChainExample:
    """Test LangChain integration example."""

    def test_example_imports(self):
        """Test that langchain_example.py can be imported."""
        try:
            import langchain_example
            assert True
        except ImportError as e:
            # langchain may not be installed
            if "langchain" in str(e):
                pytest.skip("langchain not installed")
            raise

    def test_bridge_creation(self):
        """Test that Bridge can be instantiated from example."""
        import os
        os.environ["JWT_SECRET"] = "test-secret"
        
        from agent_ros_bridge import Bridge
        bridge = Bridge()
        assert bridge is not None
        
    def test_langchain_tool_method_exists(self):
        """Test that get_langchain_tool method exists."""
        import os
        os.environ["JWT_SECRET"] = "test-secret"
        
        from agent_ros_bridge import Bridge
        bridge = Bridge()
        
        # Method should exist
        assert hasattr(bridge, 'get_langchain_tool')
        
    def test_example_structure(self):
        """Test that example has required structure."""
        example_file = examples_dir / "langchain_example.py"
        assert example_file.exists()
        
        content = example_file.read_text()
        
        # Should have main function
        assert "async def main()" in content
        
        # Should use Bridge
        assert "from agent_ros_bridge import Bridge" in content
        
        # Should set JWT_SECRET
        assert "JWT_SECRET" in content
