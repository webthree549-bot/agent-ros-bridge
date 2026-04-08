"""TDD Tests for Documentation.

Test-Driven Documentation:
1. Write tests that check documentation exists and is complete
2. Run tests - expect FAIL (docs missing/incomplete)
3. Write/update documentation
4. Verify all tests pass
"""

import os
import re
from pathlib import Path

import pytest


PROJECT_ROOT = Path("/Users/webthree/.openclaw/workspace")


class TestREADMEExistence:
    """TDD tests for README.md existence and content."""
    
    def test_readme_exists(self):
        """README.md must exist at project root."""
        readme_path = PROJECT_ROOT / "README.md"
        assert readme_path.exists(), "README.md not found at project root"
    
    def test_readme_not_empty(self):
        """README.md must not be empty."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text()
        assert len(content) > 1000, "README.md is too short (< 1000 chars)"
    
    def test_readme_has_title(self):
        """README.md must have a title."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text()
        assert "# " in content, "README.md missing title header"
    
    def test_readme_has_installation(self):
        """README.md must have installation instructions."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text().lower()
        assert "install" in content, "README.md missing installation section"
    
    def test_readme_has_quick_start(self):
        """README.md must have quick start guide."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text().lower()
        assert any(x in content for x in ["quick start", "getting started", "usage"]), \
            "README.md missing quick start/getting started section"
    
    def test_readme_has_safety_info(self):
        """README.md must mention safety features."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text().lower()
        assert "safety" in content, "README.md missing safety information"


class TestAPIDocumentation:
    """TDD tests for API documentation."""
    
    def test_agentic_module_has_docstrings(self):
        """RobotAgent class must have docstrings."""
        from agent_ros_bridge.agentic import RobotAgent
        assert RobotAgent.__doc__, "RobotAgent missing class docstring"
        assert RobotAgent.__init__.__doc__, "RobotAgent.__init__ missing docstring"
    
    def test_tools_have_docstrings(self):
        """Tool classes must have docstrings."""
        from agent_ros_bridge.tools import ROSTopicEchoTool, ROSServiceCallTool
        
        assert ROSTopicEchoTool.__doc__, "ROSTopicEchoTool missing docstring"
        assert ROSTopicEchoTool.execute.__doc__, "ROSTopicEchoTool.execute missing docstring"
        
        assert ROSServiceCallTool.__doc__, "ROSServiceCallTool missing docstring"
        assert ROSServiceCallTool.execute.__doc__, "ROSServiceCallTool.execute missing docstring"
    
    def test_exceptions_have_docstrings(self):
        """Exception classes must have docstrings."""
        from agent_ros_bridge.exceptions import (
            AgentROSBridgeError,
            RobotConnectionError,
            SafetyValidationError,
        )
        
        assert AgentROSBridgeError.__doc__, "AgentROSBridgeError missing docstring"
        assert RobotConnectionError.__doc__, "RobotConnectionError missing docstring"
        assert SafetyValidationError.__doc__, "SafetyValidationError missing docstring"


class TestExamplesDocumentation:
    """TDD tests for examples documentation."""
    
    def test_examples_directory_exists(self):
        """Examples directory must exist."""
        examples_dir = PROJECT_ROOT / "examples"
        assert examples_dir.exists(), "examples/ directory not found"
    
    def test_examples_have_readme(self):
        """Examples must have README."""
        examples_readme = PROJECT_ROOT / "examples" / "README.md"
        assert examples_readme.exists(), "examples/README.md not found"
    
    def test_ai_integrations_have_docs(self):
        """AI agent integrations must have documentation."""
        ai_integrations_dir = PROJECT_ROOT / "examples" / "ai_agent_integrations"
        if ai_integrations_dir.exists():
            readme = ai_integrations_dir / "README.md"
            assert readme.exists(), "AI integrations README.md not found"


class TestSafetyDocumentation:
    """TDD tests for safety documentation."""
    
    def test_safety_md_exists(self):
        """docs/SAFETY.md must exist."""
        safety_md = PROJECT_ROOT / "docs" / "SAFETY.md"
        assert safety_md.exists(), "docs/SAFETY.md not found"
    
    def test_safety_md_has_human_in_loop(self):
        """Safety doc must mention human-in-the-loop."""
        safety_md = PROJECT_ROOT / "docs" / "SAFETY.md"
        if safety_md.exists():
            content = safety_md.read_text().lower()
            assert "human" in content, "Safety doc missing human-in-the-loop info"
    
    def test_safety_md_has_shadow_mode(self):
        """Safety doc must mention shadow mode."""
        safety_md = PROJECT_ROOT / "docs" / "SAFETY.md"
        if safety_md.exists():
            content = safety_md.read_text().lower()
            assert "shadow" in content, "Safety doc missing shadow mode info"


class TestChangelog:
    """TDD tests for CHANGELOG."""
    
    def test_changelog_exists(self):
        """CHANGELOG.md must exist."""
        changelog = PROJECT_ROOT / "CHANGELOG.md"
        assert changelog.exists(), "CHANGELOG.md not found"
    
    def test_changelog_has_latest_version(self):
        """CHANGELOG must have latest version entry."""
        changelog = PROJECT_ROOT / "CHANGELOG.md"
        content = changelog.read_text()
        # Should have v0.6.6 or similar recent version
        assert re.search(r"v?0\.6\.[0-9]", content), "CHANGELOG missing recent version"


class TestDocstringsCoverage:
    """TDD tests for docstring coverage."""
    
    def test_public_classes_have_docstrings(self):
        """All public classes should have docstrings."""
        import agent_ros_bridge
        
        # Get all public classes from main module
        public_names = [name for name in dir(agent_ros_bridge) if not name.startswith("_")]
        
        missing_docstrings = []
        for name in public_names:
            obj = getattr(agent_ros_bridge, name)
            if isinstance(obj, type) and not obj.__doc__:
                missing_docstrings.append(name)
        
        assert len(missing_docstrings) == 0, \
            f"Classes missing docstrings: {missing_docstrings}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
