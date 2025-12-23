#!/usr/bin/env python3
"""
Validation script for Module 3: The AI-Robot Brain (NVIDIA Isaac) code examples.

This script validates:
1. Python syntax for all .py files in docs/module-3-isaac/assets/code/
2. YAML schema for all ROS 2 config files
3. USD model validity (basic checks)
4. Markdown frontmatter completeness for all chapter files

Usage:
    python scripts/validate_module3_examples.py
    python scripts/validate_module3_examples.py --path docs/module-3-isaac/assets/code/isaac_sim/
"""

import argparse
import ast
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import yaml
except ImportError:
    yaml = None
    print("Warning: PyYAML not installed. YAML validation will be skipped.")
    print("Install with: pip install pyyaml")


class ValidationResult:
    """Container for validation results."""

    def __init__(self):
        self.passed: List[str] = []
        self.failed: List[Tuple[str, str]] = []  # (file, error_message)
        self.warnings: List[Tuple[str, str]] = []  # (file, warning_message)

    def add_pass(self, file_path: str):
        """Record a passed validation."""
        self.passed.append(file_path)

    def add_fail(self, file_path: str, error: str):
        """Record a failed validation."""
        self.failed.append((file_path, error))

    def add_warning(self, file_path: str, warning: str):
        """Record a validation warning."""
        self.warnings.append((file_path, warning))

    def summary(self) -> str:
        """Generate summary report."""
        total = len(self.passed) + len(self.failed)
        lines = [
            f"\n{'='*60}",
            "VALIDATION SUMMARY",
            f"{'='*60}",
            f"Total files checked: {total}",
            f"[PASS] Passed: {len(self.passed)}",
            f"[FAIL] Failed: {len(self.failed)}",
            f"[WARN] Warnings: {len(self.warnings)}",
        ]

        if self.failed:
            lines.append(f"\n{'='*60}")
            lines.append("FAILURES:")
            lines.append(f"{'='*60}")
            for file_path, error in self.failed:
                lines.append(f"\n[FAIL] {file_path}")
                lines.append(f"   {error}")

        if self.warnings:
            lines.append(f"\n{'='*60}")
            lines.append("WARNINGS:")
            lines.append(f"{'='*60}")
            for file_path, warning in self.warnings:
                lines.append(f"\n[WARN] {file_path}")
                lines.append(f"   {warning}")

        lines.append(f"\n{'='*60}")
        return "\n".join(lines)

    def is_success(self) -> bool:
        """Check if all validations passed."""
        return len(self.failed) == 0


def validate_python_syntax(file_path: Path, result: ValidationResult):
    """Validate Python file syntax using AST parser."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()

        # Parse the Python code
        ast.parse(code, filename=str(file_path))

        # Additional checks
        if len(code.strip()) == 0:
            result.add_warning(str(file_path), "File is empty")
        elif not code.strip().startswith(('#', '"""', "'''")):
            result.add_warning(str(file_path), "Missing module docstring or shebang")

        result.add_pass(str(file_path))
        print(f"[PASS] {file_path.name}")

    except SyntaxError as e:
        result.add_fail(str(file_path), f"Syntax error at line {e.lineno}: {e.msg}")
        print(f"[FAIL] {file_path.name} - Syntax error")
    except Exception as e:
        result.add_fail(str(file_path), f"Error: {str(e)}")
        print(f"[FAIL] {file_path.name} - {str(e)}")


def validate_yaml_syntax(file_path: Path, result: ValidationResult):
    """Validate YAML file syntax."""
    if yaml is None:
        result.add_warning(str(file_path), "YAML validation skipped (PyYAML not installed)")
        return

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = yaml.safe_load(f)

        if content is None:
            result.add_warning(str(file_path), "YAML file is empty or contains only comments")

        result.add_pass(str(file_path))
        print(f"[PASS] {file_path.name}")

    except yaml.YAMLError as e:
        result.add_fail(str(file_path), f"YAML error: {str(e)}")
        print(f"[FAIL] {file_path.name} - YAML error")
    except Exception as e:
        result.add_fail(str(file_path), f"Error: {str(e)}")
        print(f"[FAIL] {file_path.name} - {str(e)}")


def validate_usd_basic(file_path: Path, result: ValidationResult):
    """Basic USD file validation (checks file structure, not deep validation)."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Basic USD file checks
        if len(content.strip()) == 0:
            result.add_fail(str(file_path), "USD file is empty")
            print(f"[FAIL] {file_path.name} - Empty file")
            return

        # Check for USD header
        if not content.startswith('#usda'):
            result.add_warning(str(file_path), "Missing USD ASCII header '#usda'")

        # Check for basic USD keywords
        required_keywords = ['def']
        missing_keywords = [kw for kw in required_keywords if kw not in content]
        if missing_keywords:
            result.add_warning(str(file_path),
                             f"Missing USD keywords: {', '.join(missing_keywords)}")

        result.add_pass(str(file_path))
        print(f"[PASS] {file_path.name}")

    except Exception as e:
        result.add_fail(str(file_path), f"Error: {str(e)}")
        print(f"[FAIL] {file_path.name} - {str(e)}")


def validate_markdown_frontmatter(file_path: Path, result: ValidationResult):
    """Validate Markdown chapter frontmatter."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Check for frontmatter
        if not content.startswith('---'):
            result.add_warning(str(file_path), "Missing frontmatter")
            return

        # Extract frontmatter
        parts = content.split('---', 2)
        if len(parts) < 3:
            result.add_fail(str(file_path), "Incomplete frontmatter (missing closing '---')")
            print(f"❌ {file_path.name} - Incomplete frontmatter")
            return

        frontmatter_text = parts[1].strip()

        # Required fields
        required_fields = ['title', 'description']
        recommended_fields = ['sidebar_position']

        missing_required = [field for field in required_fields
                          if f'{field}:' not in frontmatter_text]
        missing_recommended = [field for field in recommended_fields
                             if f'{field}:' not in frontmatter_text]

        if missing_required:
            result.add_fail(str(file_path),
                          f"Missing required frontmatter fields: {', '.join(missing_required)}")
            print(f"[FAIL] {file_path.name} - Missing required fields")
        elif missing_recommended:
            result.add_warning(str(file_path),
                             f"Missing recommended frontmatter fields: {', '.join(missing_recommended)}")
            result.add_pass(str(file_path))
            print(f"[PASS] {file_path.name} (with warnings)")
        else:
            result.add_pass(str(file_path))
            print(f"[PASS] {file_path.name}")

    except Exception as e:
        result.add_fail(str(file_path), f"Error: {str(e)}")
        print(f"[FAIL] {file_path.name} - {str(e)}")


def validate_directory(directory: Path, result: ValidationResult):
    """Validate all files in a directory."""
    print(f"\n{'='*60}")
    print(f"Validating directory: {directory}")
    print(f"{'='*60}\n")

    # Python files
    python_files = list(directory.glob('**/*.py'))
    if python_files:
        print(f"\n[Python] Python files ({len(python_files)}):")
        for py_file in sorted(python_files):
            validate_python_syntax(py_file, result)

    # YAML files
    yaml_files = list(directory.glob('**/*.yaml')) + list(directory.glob('**/*.yml'))
    if yaml_files:
        print(f"\n[YAML] YAML files ({len(yaml_files)}):")
        for yaml_file in sorted(yaml_files):
            validate_yaml_syntax(yaml_file, result)

    # USD files
    usd_files = list(directory.glob('**/*.usda')) + list(directory.glob('**/*.usd'))
    if usd_files:
        print(f"\n[USD] USD files ({len(usd_files)}):")
        for usd_file in sorted(usd_files):
            validate_usd_basic(usd_file, result)

    # Markdown chapter files
    if 'module-3-isaac' in str(directory):
        md_files = [f for f in directory.glob('ch*.md')]
        if md_files:
            print(f"\n[Markdown] Markdown chapter files ({len(md_files)}):")
            for md_file in sorted(md_files):
                validate_markdown_frontmatter(md_file, result)


def main():
    """Main validation entry point."""
    parser = argparse.ArgumentParser(
        description="Validate Module 3 code examples and assets"
    )
    parser.add_argument(
        '--path',
        type=str,
        default='docs/module-3-isaac',
        help='Path to validate (default: docs/module-3-isaac)'
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results in JSON format'
    )

    args = parser.parse_args()

    # Resolve path
    repo_root = Path(__file__).parent.parent
    validation_path = repo_root / args.path

    if not validation_path.exists():
        print(f"❌ Error: Path does not exist: {validation_path}")
        sys.exit(1)

    # Run validation
    result = ValidationResult()
    validate_directory(validation_path, result)

    # Output results
    if args.json:
        output = {
            'total': len(result.passed) + len(result.failed),
            'passed': len(result.passed),
            'failed': len(result.failed),
            'warnings': len(result.warnings),
            'failures': [{'file': f, 'error': e} for f, e in result.failed],
            'warnings_list': [{'file': f, 'warning': w} for f, w in result.warnings],
        }
        print(json.dumps(output, indent=2))
    else:
        print(result.summary())

    # Exit code
    sys.exit(0 if result.is_success() else 1)


if __name__ == '__main__':
    main()
