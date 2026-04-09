"""Enhanced TDD Tests for Documentation Improvements.

These tests drive documentation quality improvements.
"""

import os
import re
from pathlib import Path

import pytest

PROJECT_ROOT = Path("/Users/webthree/.openclaw/workspace")


class TestREADMEEnhancements:
    """TDD tests for README enhancements."""

    def test_readme_has_badge_section(self):
        """README should have badges (CI, version, etc.)."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text()
        # Check for common badge patterns
        has_badge = any(x in content for x in ["![", "[badge", "shields.io"])
        assert has_badge, "README missing badges section"

    def test_readme_has_architecture_diagram(self):
        """README should describe architecture."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text().lower()
        assert any(
            x in content for x in ["architecture", "overview", "design", "diagram"]
        ), "README missing architecture overview"

    def test_readme_has_ai_agent_section(self):
        """README should mention AI agent integrations."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text().lower()
        assert any(
            x in content for x in ["ai agent", "llm", "langchain", "openclaw"]
        ), "README missing AI agent integration section"

    def test_readme_has_feature_list(self):
        """README should have clear feature list."""
        readme_path = PROJECT_ROOT / "README.md"
        content = readme_path.read_text()
        # Look for feature lists with checkmarks or bullet points
        has_features = bool(re.search(r"[-*]\s+\w+", content))
        assert has_features, "README missing feature list"


class TestToolsDocumentation:
    """TDD tests for tools documentation."""

    def test_tools_directory_has_readme(self):
        """tools/ directory should have README."""
        tools_readme = PROJECT_ROOT / "agent_ros_bridge" / "tools" / "README.md"
        assert tools_readme.exists(), "tools/README.md not found"

    def test_tools_readme_has_examples(self):
        """Tools README should have usage examples."""
        tools_readme = PROJECT_ROOT / "agent_ros_bridge" / "tools" / "README.md"
        if tools_readme.exists():
            content = tools_readme.read_text().lower()
            assert "usage" in content or "example" in content, "Tools README missing usage examples"

    def test_tools_readme_has_nasa_rosa_compat(self):
        """Tools README should mention NASA ROSA compatibility."""
        tools_readme = PROJECT_ROOT / "agent_ros_bridge" / "tools" / "README.md"
        if tools_readme.exists():
            content = tools_readme.read_text().lower()
            assert (
                "nasa" in content or "rosa" in content
            ), "Tools README missing NASA ROSA compatibility info"


class TestContributingDocumentation:
    """TDD tests for contributing guidelines."""

    def test_contributing_md_exists(self):
        """CONTRIBUTING.md should exist."""
        contributing = PROJECT_ROOT / "CONTRIBUTING.md"
        assert contributing.exists(), "CONTRIBUTING.md not found"

    def test_contributing_has_tdd_guidelines(self):
        """CONTRIBUTING should mention TDD."""
        contributing = PROJECT_ROOT / "CONTRIBUTING.md"
        if contributing.exists():
            content = contributing.read_text().lower()
            assert (
                "tdd" in content or "test driven" in content
            ), "CONTRIBUTING missing TDD guidelines"


class TestAPIDocsDetailed:
    """TDD tests for detailed API documentation."""

    def test_robot_agent_has_usage_example_in_docstring(self):
        """RobotAgent docstring should have usage example."""
        from agent_ros_bridge.agentic import RobotAgent

        docstring = RobotAgent.__doc__ or ""
        # Check for code block or example
        has_example = "example" in docstring.lower() or "```" in docstring
        assert has_example, "RobotAgent missing usage example in docstring"

    def test_tool_execute_has_param_documentation(self):
        """Tool.execute should document all parameters."""
        from agent_ros_bridge.tools import ROSTopicEchoTool

        docstring = ROSTopicEchoTool.execute.__doc__ or ""
        # Should document 'topic' parameter
        assert "topic" in docstring, "ROSTopicEchoTool.execute missing 'topic' param docs"


class TestDocsDirectoryStructure:
    """TDD tests for docs/ directory structure."""

    def test_docs_has_api_directory(self):
        """docs/ should have api/ subdirectory."""
        api_docs = PROJECT_ROOT / "docs" / "api"
        assert api_docs.exists(), "docs/api/ directory not found"

    def test_docs_has_examples_directory(self):
        """docs/ should have examples/ subdirectory."""
        examples_docs = PROJECT_ROOT / "docs" / "examples"
        assert examples_docs.exists(), "docs/examples/ directory not found"

    def test_docs_has_guides_directory(self):
        """docs/ should have guides/ subdirectory."""
        guides_docs = PROJECT_ROOT / "docs" / "guides"
        assert guides_docs.exists(), "docs/guides/ directory not found"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
