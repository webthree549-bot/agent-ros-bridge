"""Package the Agent ROS Bridge skill for ClawHub distribution.

Usage:
    python scripts/package_skill.py [output_dir]
"""

import argparse
import sys
import zipfile
from pathlib import Path


def validate_skill(skill_dir: Path) -> list[str]:
    """Validate skill structure before packaging.

    Returns:
        List of error messages (empty if valid)
    """
    errors = []

    # Check SKILL.md exists
    skill_md = skill_dir / "SKILL.md"
    if not skill_md.exists():
        errors.append("SKILL.md not found")
        return errors

    # Check YAML frontmatter
    content = skill_md.read_text()
    if not content.startswith("---"):
        errors.append("SKILL.md must start with YAML frontmatter")

    # Check for forbidden files
    forbidden = ["README.md", "CHANGELOG.md", "INSTALLATION_GUIDE.md"]
    for fname in forbidden:
        if (skill_dir / fname).exists():
            errors.append(f"Forbidden file found: {fname}")

    return errors


def package_skill(skill_dir: Path, output_dir: Path) -> Path:
    """Package skill into .skill file (zip format).

    Args:
        skill_dir: Path to skill directory
        output_dir: Path to output directory

    Returns:
        Path to created .skill file
    """
    skill_name = skill_dir.name
    output_file = output_dir / f"{skill_name}.skill"

    # Create zip file
    with zipfile.ZipFile(output_file, "w", zipfile.ZIP_DEFLATED) as zf:
        for file_path in skill_dir.rglob("*"):
            if file_path.is_file():
                # Add file to zip, preserving relative structure
                arcname = file_path.relative_to(skill_dir)
                zf.write(file_path, arcname)

    return output_file


def main():
    parser = argparse.ArgumentParser(description="Package Agent ROS Bridge skill")
    parser.add_argument(
        "output_dir",
        nargs="?",
        default=".",
        help="Output directory for .skill file (default: current directory)",
    )

    args = parser.parse_args()

    # Find skill directory
    script_dir = Path(__file__).parent
    skill_dir = script_dir.parent  # skills/agent-ros-bridge

    print(f"Packaging skill from: {skill_dir}")

    # Validate
    print("Validating skill...")
    errors = validate_skill(skill_dir)
    if errors:
        print("Validation failed:")
        for error in errors:
            print(f"  - {error}")
        sys.exit(1)

    print("Validation passed!")

    # Package
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Packaging to: {output_dir}")
    output_file = package_skill(skill_dir, output_dir)

    print(f"Created: {output_file}")
    print(f"Size: {output_file.stat().st_size} bytes")

    # List contents
    print("\nPackage contents:")
    with zipfile.ZipFile(output_file, "r") as zf:
        for info in zf.infolist():
            print(f"  {info.filename} ({info.file_size} bytes)")


if __name__ == "__main__":
    main()
