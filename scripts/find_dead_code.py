#!/usr/bin/env python3
"""
Find potentially dead code in agent_ros_bridge.

Usage:
    python scripts/find_dead_code.py
"""

import ast
import os
import sys
from pathlib import Path
from collections import defaultdict


def find_python_files(directory):
    """Find all Python files in directory."""
    files = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            if filename.endswith('.py') and not filename.startswith('__'):
                files.append(Path(root) / filename)
    return files


def extract_imports_and_definitions(filepath):
    """Extract imports and definitions from a Python file."""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())
    except SyntaxError:
        return set(), set(), set()
    
    imports = set()
    definitions = set()
    exports = set()
    
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ''
            for alias in node.names:
                if node.module:
                    imports.add(f"{node.module}.{alias.name}")
        elif isinstance(node, ast.FunctionDef):
            definitions.add(node.name)
        elif isinstance(node, ast.ClassDef):
            definitions.add(node.name)
    
    # Check for __all__ exports
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == '__all__':
                    if isinstance(node.value, ast.List):
                        for elt in node.value.elts:
                            if isinstance(elt, ast.Constant):
                                exports.add(elt.value)
    
    return imports, definitions, exports


def find_unused_definitions(package_path):
    """Find definitions that might be unused."""
    all_files = find_python_files(package_path)
    
    # Map of file -> definitions
    file_definitions = {}
    # Map of name -> list of files that define it
    name_to_files = defaultdict(list)
    # All imports across all files
    all_imports = set()
    # All exports across all files
    all_exports = set()
    
    for filepath in all_files:
        imports, definitions, exports = extract_imports_and_definitions(filepath)
        file_definitions[filepath] = definitions
        for name in definitions:
            name_to_files[name].append(filepath)
        all_imports.update(imports)
        all_exports.update(exports)
    
    # Find potentially unused definitions
    potentially_unused = []
    
    for filepath, definitions in file_definitions.items():
        for name in definitions:
            # Skip if exported
            if name in all_exports:
                continue
            
            # Skip if imported elsewhere
            full_names = [
                f"agent_ros_bridge.{filepath.stem}.{name}",
                f"agent_ros_bridge.{name}",
                name
            ]
            
            imported = any(imp.endswith(name) or name in imp for imp in all_imports)
            
            if not imported and len(name_to_files[name]) == 1:
                potentially_unused.append((filepath, name))
    
    return potentially_unused


def main():
    package_path = Path(__file__).parent.parent / 'agent_ros_bridge'
    
    if not package_path.exists():
        print(f"Error: {package_path} not found")
        sys.exit(1)
    
    print(f"Scanning {package_path} for dead code...\n")
    
    unused = find_unused_definitions(package_path)
    
    if unused:
        print(f"Found {len(unused)} potentially unused definitions:\n")
        
        # Group by file
        by_file = defaultdict(list)
        for filepath, name in unused:
            by_file[filepath].append(name)
        
        for filepath, names in sorted(by_file.items()):
            print(f"{filepath}:")
            for name in sorted(names):
                print(f"  - {name}")
            print()
    else:
        print("No obviously unused definitions found.")
    
    # Find duplicate file patterns
    print("\n" + "="*60)
    print("CHECKING FOR POTENTIAL DUPLICATES:\n")
    
    all_files = find_python_files(package_path)
    stem_counts = defaultdict(list)
    for f in all_files:
        stem_counts[f.stem].append(f)
    
    for stem, files in stem_counts.items():
        if len(files) > 1:
            print(f"'{stem}' appears in multiple files:")
            for f in files:
                print(f"  - {f}")
            print()


if __name__ == '__main__':
    main()
