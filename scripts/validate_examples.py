#!/usr/bin/env python3
"""
Validation script for Python code examples in Markdown files.

This script:
1. Extracts Python code blocks from Markdown files
2. Validates syntax using AST parsing
3. Checks for common issues (imports, ROS 2 patterns)
4. Reports errors with line numbers and file locations
"""

import ast
import re
import sys
from pathlib import Path
from typing import List, Dict, Tuple


class CodeBlock:
    """Represents a code block extracted from Markdown."""

    def __init__(self, code: str, language: str, file_path: Path, line_num: int):
        self.code = code
        self.language = language
        self.file_path = file_path
        self.line_num = line_num


def extract_code_blocks(markdown_path: Path) -> List[CodeBlock]:
    """Extract all code blocks from a Markdown file."""
    code_blocks = []

    with open(markdown_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern matches fenced code blocks with optional language identifier
    pattern = r'```(\w+)?\n(.*?)```'

    for match in re.finditer(pattern, content, re.DOTALL):
        language = match.group(1) or 'text'
        code = match.group(2)

        # Calculate line number of code block start
        line_num = content[:match.start()].count('\n') + 1

        code_blocks.append(CodeBlock(code, language, markdown_path, line_num))

    return code_blocks


def validate_python_syntax(code_block: CodeBlock) -> Tuple[bool, str]:
    """Validate Python code syntax using AST parsing."""
    try:
        ast.parse(code_block.code)
        return True, ""
    except SyntaxError as e:
        error_msg = f"Syntax error at line {e.lineno}: {e.msg}"
        return False, error_msg
    except Exception as e:
        return False, f"Parse error: {str(e)}"


def check_ros2_imports(code_block: CodeBlock) -> List[str]:
    """Check for common ROS 2 import patterns and potential issues."""
    warnings = []

    # Check for rclpy usage without import
    if 'rclpy.' in code_block.code and 'import rclpy' not in code_block.code:
        warnings.append("Uses rclpy but missing 'import rclpy'")

    # Check for Node usage without proper import
    if 'Node(' in code_block.code and 'from rclpy.node import Node' not in code_block.code:
        if 'import rclpy' not in code_block.code or 'rclpy.node.Node' not in code_block.code:
            warnings.append("Uses Node but missing proper import")

    # Check for common publisher/subscriber patterns
    if 'create_publisher' in code_block.code or 'create_subscription' in code_block.code:
        if 'std_msgs' not in code_block.code and 'from' not in code_block.code:
            warnings.append("Uses publishers/subscribers but no message type imports detected")

    return warnings


def validate_file(markdown_path: Path, verbose: bool = False) -> Tuple[int, int, List[str]]:
    """
    Validate all Python code blocks in a Markdown file.

    Returns:
        Tuple of (total_blocks, error_count, error_messages)
    """
    code_blocks = extract_code_blocks(markdown_path)
    python_blocks = [b for b in code_blocks if b.language in ('python', 'py')]

    if not python_blocks:
        if verbose:
            print(f"  No Python code blocks found in {markdown_path}")
        return 0, 0, []

    errors = []
    error_count = 0

    for block in python_blocks:
        # Validate syntax
        is_valid, error_msg = validate_python_syntax(block)

        if not is_valid:
            error_count += 1
            errors.append(
                f"{block.file_path}:{block.line_num} - {error_msg}"
            )

        # Check ROS 2 patterns
        warnings = check_ros2_imports(block)
        if warnings:
            for warning in warnings:
                errors.append(
                    f"{block.file_path}:{block.line_num} - Warning: {warning}"
                )

    if verbose:
        print(f"  Validated {len(python_blocks)} Python blocks in {markdown_path}")

    return len(python_blocks), error_count, errors


def main():
    """Main validation routine."""
    import argparse

    parser = argparse.ArgumentParser(description='Validate Python code examples in Markdown files')
    parser.add_argument('paths', nargs='*', help='Paths to Markdown files or directories')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--docs-dir', default='docs', help='Documentation directory (default: docs)')

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
    total_blocks = 0
    total_errors = 0
    all_errors = []

    print(f"Validating {len(files_to_validate)} Markdown files...")

    for md_file in sorted(files_to_validate):
        blocks, errors, messages = validate_file(md_file, verbose=args.verbose)
        total_blocks += blocks
        total_errors += errors
        all_errors.extend(messages)

    # Report results
    print(f"\n{'='*60}")
    print(f"Validation Results:")
    print(f"  Files checked: {len(files_to_validate)}")
    print(f"  Python blocks validated: {total_blocks}")
    print(f"  Errors found: {total_errors}")
    print(f"{'='*60}")

    if all_errors:
        print("\nErrors and warnings:")
        for error in all_errors:
            print(f"  {error}")
        return 1
    else:
        print("\n[PASS] All Python code examples are valid!")
        return 0


if __name__ == '__main__':
    sys.exit(main())
