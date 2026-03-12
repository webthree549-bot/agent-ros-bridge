"""Test skill packaging for ClawHub distribution.

TDD for packaging script.
"""

import tempfile
import unittest
import zipfile
from pathlib import Path

SKILL_PATH = Path(__file__).parent.parent.parent / "skills" / "agent-ros-bridge"
PACKAGE_SCRIPT = SKILL_PATH / "scripts" / "package_skill.py"


class TestPackagingScript(unittest.TestCase):
    """Test skill packaging functionality."""

    def test_package_script_exists(self):
        """Test that packaging script exists."""
        self.assertTrue(PACKAGE_SCRIPT.exists(), f"Package script not found: {PACKAGE_SCRIPT}")

    def test_package_script_is_valid_python(self):
        """Test that packaging script is valid Python."""
        import ast

        try:
            ast.parse(PACKAGE_SCRIPT.read_text())
        except SyntaxError as e:
            self.fail(f"Package script has syntax error: {e}")

    def test_skill_validates_successfully(self):
        """Test that skill passes validation."""
        # Import and test validation function
        import sys

        sys.path.insert(0, str(SKILL_PATH / "scripts"))

        try:
            from package_skill import validate_skill

            errors = validate_skill(SKILL_PATH)
            self.assertEqual(errors, [], f"Skill validation failed: {errors}")
        finally:
            sys.path.pop(0)

    def test_skill_packages_successfully(self):
        """Test that skill can be packaged."""
        import sys

        sys.path.insert(0, str(SKILL_PATH / "scripts"))

        try:
            from package_skill import package_skill, validate_skill

            # Validate first
            errors = validate_skill(SKILL_PATH)
            if errors:
                self.skipTest(f"Skill not valid: {errors}")

            # Package to temp directory
            with tempfile.TemporaryDirectory() as tmpdir:
                output_dir = Path(tmpdir)
                output_file = package_skill(SKILL_PATH, output_dir)

                # Check output file exists
                self.assertTrue(output_file.exists(), "Package file not created")

                # Check it's a valid zip
                self.assertTrue(zipfile.is_zipfile(output_file), "Output is not a valid zip file")

                # Check contents
                with zipfile.ZipFile(output_file, "r") as zf:
                    files = zf.namelist()

                    # Must have SKILL.md
                    self.assertIn("SKILL.md", files, "Package must contain SKILL.md")

        finally:
            sys.path.pop(0)

    def test_package_has_correct_structure(self):
        """Test that packaged skill has correct structure."""
        import sys

        sys.path.insert(0, str(SKILL_PATH / "scripts"))

        try:
            from package_skill import package_skill, validate_skill

            errors = validate_skill(SKILL_PATH)
            if errors:
                self.skipTest(f"Skill not valid: {errors}")

            with tempfile.TemporaryDirectory() as tmpdir:
                output_dir = Path(tmpdir)
                output_file = package_skill(SKILL_PATH, output_dir)

                with zipfile.ZipFile(output_file, "r") as zf:
                    files = zf.namelist()

                    # Check required files
                    self.assertIn("SKILL.md", files)

                    # Check no forbidden files
                    forbidden = ["README.md", "CHANGELOG.md"]
                    for f in forbidden:
                        self.assertNotIn(f, files, f"Package should not contain {f}")

        finally:
            sys.path.pop(0)


if __name__ == "__main__":
    unittest.main()
