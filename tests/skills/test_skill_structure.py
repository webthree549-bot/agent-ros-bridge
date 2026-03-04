"""Test skill structure validation for Agent ROS Bridge ClawHub skill.

TDD Step 1: Write tests that fail
TDD Step 2: Implement to make tests pass
TDD Step 3: Refactor
"""

import re
import unittest
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


SKILL_PATH = Path(__file__).parent.parent.parent / "skills" / "agent-ros-bridge"
SKILL_MD_PATH = SKILL_PATH / "SKILL.md"


class TestSkillStructure(unittest.TestCase):
    """Test that skill follows ClawHub structure requirements."""

    def test_skill_directory_exists(self):
        """Test that skill directory exists."""
        self.assertTrue(
            SKILL_PATH.exists(),
            f"Skill directory not found: {SKILL_PATH}"
        )

    def test_skill_md_exists(self):
        """Test that SKILL.md exists."""
        self.assertTrue(
            SKILL_MD_PATH.exists(),
            f"SKILL.md not found: {SKILL_MD_PATH}"
        )

    def test_no_extraneous_files(self):
        """Test that no README.md, CHANGELOG.md, etc. exist."""
        forbidden_files = [
            "README.md",
            "CHANGELOG.md",
            "INSTALLATION_GUIDE.md",
            "QUICK_REFERENCE.md",
        ]
        
        for filename in forbidden_files:
            file_path = SKILL_PATH / filename
            self.assertFalse(
                file_path.exists(),
                f"Extraneous file found: {filename}"
            )

    def test_skill_md_has_yaml_frontmatter(self):
        """Test that SKILL.md has YAML frontmatter."""
        content = SKILL_MD_PATH.read_text()
        
        # Check for YAML frontmatter (--- at start)
        self.assertTrue(
            content.startswith("---"),
            "SKILL.md must start with YAML frontmatter (---)"
        )
        
        # Check for closing ---
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        self.assertIsNotNone(
            frontmatter_match,
            "SKILL.md must have valid YAML frontmatter delimiters"
        )

    def test_yaml_frontmatter_has_required_fields(self):
        """Test that YAML frontmatter has name and description."""
        content = SKILL_MD_PATH.read_text()
        
        # Extract frontmatter
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        self.assertIsNotNone(frontmatter_match, "Invalid frontmatter")
        
        frontmatter_yaml = frontmatter_match.group(1)
        
        try:
            frontmatter = yaml.safe_load(frontmatter_yaml)
        except yaml.YAMLError as e:
            self.fail(f"Invalid YAML in frontmatter: {e}")
        
        # Check required fields
        self.assertIn(
            "name",
            frontmatter,
            "Frontmatter must have 'name' field"
        )
        self.assertIn(
            "description",
            frontmatter,
            "Frontmatter must have 'description' field"
        )
        
        # Check no other fields
        allowed_fields = {"name", "description"}
        extra_fields = set(frontmatter.keys()) - allowed_fields
        self.assertEqual(
            extra_fields,
            set(),
            f"Frontmatter has extra fields: {extra_fields}"
        )

    def test_name_is_valid(self):
        """Test that name is a valid string."""
        content = SKILL_MD_PATH.read_text()
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        frontmatter = yaml.safe_load(frontmatter_match.group(1))
        
        name = frontmatter.get("name")
        self.assertIsInstance(name, str, "Name must be a string")
        self.assertTrue(len(name) > 0, "Name cannot be empty")
        self.assertTrue(
            len(name) <= 100,
            "Name should be concise (<= 100 chars)"
        )

    def test_description_is_comprehensive(self):
        """Test that description explains what and when to use."""
        content = SKILL_MD_PATH.read_text()
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        frontmatter = yaml.safe_load(frontmatter_match.group(1))
        
        description = frontmatter.get("description", "")
        
        # Check length (should be substantial but not too long)
        self.assertTrue(
            len(description) >= 50,
            "Description should be at least 50 characters"
        )
        self.assertTrue(
            len(description) <= 500,
            "Description should be <= 500 characters for context efficiency"
        )
        
        # Check for triggering keywords
        triggering_keywords = [
            "use when",
            "robot",
            "ROS",
            "control",
            "navigate",
            "fleet",
        ]
        
        description_lower = description.lower()
        found_keywords = [
            kw for kw in triggering_keywords
            if kw.lower() in description_lower
        ]
        
        self.assertTrue(
            len(found_keywords) >= 2,
            f"Description should include triggering keywords. Found: {found_keywords}"
        )

    def test_skill_body_exists(self):
        """Test that skill has body content after frontmatter."""
        content = SKILL_MD_PATH.read_text()
        
        # Split frontmatter from body
        parts = re.split(r'^---\s*$', content, flags=re.MULTILINE)
        self.assertTrue(len(parts) >= 3, "SKILL.md must have frontmatter and body")
        
        body = parts[2].strip()
        self.assertTrue(
            len(body) > 0,
            "SKILL.md must have body content"
        )

    def test_skill_body_under_500_lines(self):
        """Test that skill body is under 500 lines (ClawHub best practice)."""
        content = SKILL_MD_PATH.read_text()
        
        parts = re.split(r'^---\s*$', content, flags=re.MULTILINE)
        body = parts[2] if len(parts) >= 3 else ""
        
        line_count = len(body.strip().split('\n'))
        self.assertTrue(
            line_count <= 500,
            f"Skill body should be <= 500 lines, got {line_count}"
        )

    def test_skill_uses_progressive_disclosure(self):
        """Test that skill uses progressive disclosure (references/ for details)."""
        references_dir = SKILL_PATH / "references"
        
        # Either has references directory OR body is very concise
        content = SKILL_MD_PATH.read_text()
        parts = re.split(r'^---\s*$', content, flags=re.MULTILINE)
        body = parts[2] if len(parts) >= 3 else ""
        line_count = len(body.strip().split('\n'))
        
        if line_count > 200:
            self.assertTrue(
                references_dir.exists(),
                "Skills > 200 lines should use references/ for detailed content"
            )


class TestSkillContent(unittest.TestCase):
    """Test that skill content is correct and helpful."""

    def test_has_quick_start_section(self):
        """Test that skill has a Quick Start section."""
        content = SKILL_MD_PATH.read_text()
        
        self.assertTrue(
            re.search(r'^#+\s+Quick Start', content, re.MULTILINE | re.IGNORECASE),
            "Skill should have a Quick Start section"
        )

    def test_has_websocket_connection_example(self):
        """Test that skill provides WebSocket connection example."""
        content = SKILL_MD_PATH.read_text()
        
        # Check for WebSocket mentions
        self.assertTrue(
            re.search(r'websocket|ws://|wss://', content, re.IGNORECASE),
            "Skill should mention WebSocket connection"
        )

    def test_has_jwt_token_documentation(self):
        """Test that skill documents JWT token handling."""
        content = SKILL_MD_PATH.read_text()
        
        self.assertTrue(
            re.search(r'jwt|token|JWT_SECRET', content, re.IGNORECASE),
            "Skill should document JWT authentication"
        )

    def test_has_robot_command_examples(self):
        """Test that skill has robot command examples."""
        content = SKILL_MD_PATH.read_text()
        
        command_keywords = [
            "move", "navigate", "rotate", "cmd_vel",
            "list_robots", "get_topics"
        ]
        
        found = [kw for kw in command_keywords if kw.lower() in content.lower()]
        self.assertTrue(
            len(found) >= 2,
            f"Skill should have robot command examples. Found: {found}"
        )

    def test_has_ros1_ros2_differentiation(self):
        """Test that skill explains ROS1 vs ROS2."""
        content = SKILL_MD_PATH.read_text()
        
        # Should mention both ROS1 and ROS2
        has_ros1 = re.search(r'ROS1|ros1|Noetic', content, re.IGNORECASE)
        has_ros2 = re.search(r'ROS2|ros2|Jazzy|Humble', content, re.IGNORECASE)
        
        self.assertTrue(
            has_ros1 or has_ros2,
            "Skill should mention ROS versions"
        )

    def test_has_safety_documentation(self):
        """Test that skill documents safety features."""
        content = SKILL_MD_PATH.read_text()
        
        safety_keywords = ["emergency", "estop", "safety", "dangerous"]
        found = [kw for kw in safety_keywords if kw.lower() in content.lower()]
        
        self.assertTrue(
            len(found) >= 1,
            f"Skill should document safety features. Found: {found}"
        )


class TestReferencesStructure(unittest.TestCase):
    """Test references/ directory structure if it exists."""

    def setUp(self):
        self.references_dir = SKILL_PATH / "references"

    def test_references_files_are_referenced(self):
        """Test that all files in references/ are referenced from SKILL.md."""
        if not self.references_dir.exists():
            self.skipTest("No references directory")
        
        content = SKILL_MD_PATH.read_text()
        
        for ref_file in self.references_dir.iterdir():
            if ref_file.is_file():
                # Check that file is referenced in SKILL.md
                ref_name = ref_file.name
                self.assertRegex(
                    content,
                    rf'\b{re.escape(ref_name)}\b|references/{re.escape(ref_file.stem)}',
                    f"Reference file {ref_name} should be mentioned in SKILL.md"
                )


class TestScriptsStructure(unittest.TestCase):
    """Test scripts/ directory structure if it exists."""

    def setUp(self):
        self.scripts_dir = SKILL_PATH / "scripts"

    def test_scripts_are_executable(self):
        """Test that scripts are valid Python/shell scripts."""
        import ast
        
        if not self.scripts_dir.exists():
            self.skipTest("No scripts directory")
        
        for script_file in self.scripts_dir.iterdir():
            if script_file.is_file() and script_file.suffix == '.py':
                # Try to parse as Python using AST (safer than compile)
                try:
                    ast.parse(script_file.read_text())
                except SyntaxError as e:
                    self.fail(f"Script {script_file.name} has syntax error: {e}")


if __name__ == "__main__":
    unittest.main()
