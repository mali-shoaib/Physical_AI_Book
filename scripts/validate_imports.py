#!/usr/bin/env python3
"""
Validation script for Python imports in code examples.

This script:
1. Extracts Python code blocks from Markdown files
2. Identifies all import statements
3. Checks if imports are available in the target environment
4. Validates ROS 2 package imports
5. Reports missing or unavailable imports
"""

import ast
import re
import sys
import importlib.util
from pathlib import Path
from typing import List, Set, Tuple, Dict


class ImportInfo:
    """Information about an import statement."""

    def __init__(self, module: str, names: List[str], file_path: Path, line_num: int):
        self.module = module
        self.names = names  # Empty for 'import X', populated for 'from X import Y'
        self.file_path = file_path
        self.line_num = line_num


def extract_imports_from_code(code: str, file_path: Path, code_line_num: int) -> List[ImportInfo]:
    """Extract import statements from Python code using AST."""
    imports = []

    try:
        tree = ast.parse(code)

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.append(
                        ImportInfo(
                            module=alias.name,
                            names=[],
                            file_path=file_path,
                            line_num=code_line_num + (node.lineno - 1)
                        )
                    )

            elif isinstance(node, ast.ImportFrom):
                if node.module:  # Skip relative imports without module
                    names = [alias.name for alias in node.names]
                    imports.append(
                        ImportInfo(
                            module=node.module,
                            names=names,
                            file_path=file_path,
                            line_num=code_line_num + (node.lineno - 1)
                        )
                    )

    except SyntaxError:
        # Syntax errors are handled by validate_examples.py
        pass

    return imports


def extract_python_blocks(markdown_path: Path) -> List[Tuple[str, int]]:
    """Extract Python code blocks with their line numbers."""
    blocks = []

    with open(markdown_path, 'r', encoding='utf-8') as f:
        content = f.read()

    pattern = r'```(\w+)?\n(.*?)```'

    for match in re.finditer(pattern, content, re.DOTALL):
        language = match.group(1) or 'text'
        code = match.group(2)

        if language in ('python', 'py'):
            line_num = content[:match.start()].count('\n') + 1
            blocks.append((code, line_num))

    return blocks


def is_module_available(module_name: str) -> bool:
    """Check if a module is available (can be imported)."""
    # Handle submodule checks (e.g., rclpy.node -> check rclpy)
    base_module = module_name.split('.')[0]

    # Special case: ROS 2 packages that may not be installed
    ros2_packages = {
        'rclpy', 'rclcpp', 'std_msgs', 'geometry_msgs', 'sensor_msgs',
        'nav_msgs', 'tf2_ros', 'tf2_geometry_msgs', 'urdf_parser_py',
        'robot_state_publisher', 'joint_state_publisher', 'xacro'
    }

    if base_module in ros2_packages:
        # Don't actually try to import ROS 2 packages (may not be in PATH)
        # Just validate they're known ROS 2 packages
        return True

    # Check standard library and installed packages
    spec = importlib.util.find_spec(base_module)
    return spec is not None


def get_ros2_package_category(module_name: str) -> str:
    """Categorize ROS 2 packages."""
    base_module = module_name.split('.')[0]

    categories = {
        'core': ['rclpy', 'rclcpp'],
        'messages': ['std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs'],
        'transforms': ['tf2_ros', 'tf2_geometry_msgs'],
        'description': ['urdf_parser_py', 'xacro'],
        'tools': ['robot_state_publisher', 'joint_state_publisher']
    }

    for category, packages in categories.items():
        if base_module in packages:
            return category

    return 'other'


def validate_file(markdown_path: Path, verbose: bool = False) -> Tuple[int, Dict[str, List[ImportInfo]]]:
    """
    Validate imports in all Python code blocks in a Markdown file.

    Returns:
        Tuple of (total_imports, import_issues)
        where import_issues is a dict mapping issue type to list of ImportInfo
    """
    python_blocks = extract_python_blocks(markdown_path)

    if not python_blocks:
        if verbose:
            print(f"  No Python code blocks found in {markdown_path}")
        return 0, {}

    all_imports = []

    for code, line_num in python_blocks:
        imports = extract_imports_from_code(code, markdown_path, line_num)
        all_imports.extend(imports)

    if not all_imports:
        if verbose:
            print(f"  No imports found in {markdown_path}")
        return 0, {}

    # Categorize imports
    issues = {
        'unavailable': [],
        'ros2': [],
        'standard': [],
        'third_party': []
    }

    for imp in all_imports:
        if not is_module_available(imp.module):
            # Check if it's a known ROS 2 package
            if get_ros2_package_category(imp.module) != 'other':
                issues['ros2'].append(imp)
            else:
                issues['unavailable'].append(imp)
        else:
            # Categorize available imports
            base_module = imp.module.split('.')[0]
            if base_module in sys.stdlib_module_names:
                issues['standard'].append(imp)
            else:
                issues['third_party'].append(imp)

    if verbose:
        print(f"  Found {len(all_imports)} imports in {markdown_path}")
        if issues['ros2']:
            print(f"    ROS 2 packages: {len(issues['ros2'])}")
        if issues['unavailable']:
            print(f"    Unavailable: {len(issues['unavailable'])}")

    return len(all_imports), issues


def main():
    """Main validation routine."""
    import argparse

    parser = argparse.ArgumentParser(description='Validate Python imports in Markdown files')
    parser.add_argument('paths', nargs='*', help='Paths to Markdown files or directories')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--docs-dir', default='docs', help='Documentation directory (default: docs)')
    parser.add_argument('--show-ros2', action='store_true', help='Show ROS 2 package usage')
    parser.add_argument('--strict', action='store_true', help='Fail on any unavailable imports')

    args = parser.parse_args()

    # Determine files to validate
    files_to_validate = []

    if args.paths:
        for path_str in args.paths:
            path = Path(path_str)
            if path.is_file():
                files_to_validate.append(path)
            elif path.is_dir():
                files_to_validate.extend(path.rglob('*.md'))
    else:
        # Default: validate all markdown files in docs directory
        docs_path = Path(args.docs_dir)
        if docs_path.exists():
            files_to_validate.extend(docs_path.rglob('*.md'))
        else:
            print(f"Error: Documentation directory '{args.docs_dir}' not found", file=sys.stderr)
            return 1

    if not files_to_validate:
        print("No Markdown files found to validate", file=sys.stderr)
        return 1

    # Run validation
    total_imports = 0
    all_unavailable = []
    all_ros2 = []

    print(f"Validating imports in {len(files_to_validate)} Markdown files...")

    for md_file in sorted(files_to_validate):
        count, issues = validate_file(md_file, verbose=args.verbose)
        total_imports += count
        all_unavailable.extend(issues.get('unavailable', []))
        all_ros2.extend(issues.get('ros2', []))

    # Report results
    print(f"\n{'='*60}")
    print(f"Import Validation Results:")
    print(f"  Files checked: {len(files_to_validate)}")
    print(f"  Total imports found: {total_imports}")
    print(f"  ROS 2 packages: {len(all_ros2)}")
    print(f"  Unavailable imports: {len(all_unavailable)}")
    print(f"{'='*60}")

    if args.show_ros2 and all_ros2:
        print("\nROS 2 Packages Used:")
        ros2_modules = sorted(set(imp.module for imp in all_ros2))
        for module in ros2_modules:
            category = get_ros2_package_category(module)
            print(f"  {module} ({category})")

    if all_unavailable:
        print("\nUnavailable Imports:")
        for imp in all_unavailable:
            if imp.names:
                import_str = f"from {imp.module} import {', '.join(imp.names)}"
            else:
                import_str = f"import {imp.module}"
            print(f"  {imp.file_path}:{imp.line_num} - {import_str}")

        if args.strict:
            return 1

    if not all_unavailable or not args.strict:
        print("\n[PASS] Import validation complete!")
        return 0


if __name__ == '__main__':
    sys.exit(main())
