#!/usr/bin/env python3
"""
Test script for extracting and executing Python code examples.

This script:
1. Extracts Python code blocks marked as executable
2. Runs them in isolated environments
3. Captures output and validates results
4. Reports test results with line numbers

Note: This is a basic test runner. For ROS 2 code that requires
a running ROS 2 system, use Docker-based testing (Phase 9).
"""

import ast
import re
import sys
import subprocess
import tempfile
from pathlib import Path
from typing import List, Dict, Tuple, Optional


class ExecutableBlock:
    """Represents an executable code block from Markdown."""

    def __init__(
        self,
        code: str,
        file_path: Path,
        line_num: int,
        test_id: Optional[str] = None,
        expected_output: Optional[str] = None
    ):
        self.code = code
        self.file_path = file_path
        self.line_num = line_num
        self.test_id = test_id or f"{file_path.stem}_{line_num}"
        self.expected_output = expected_output


def extract_executable_blocks(markdown_path: Path) -> List[ExecutableBlock]:
    """
    Extract executable Python code blocks from Markdown.

    Looks for blocks marked with special comments:
    ```python
    # TEST: test_name
    # EXPECTED_OUTPUT: "Hello World"
    print("Hello World")
    ```
    """
    blocks = []

    with open(markdown_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern for fenced code blocks
    pattern = r'```(python|py)\n(.*?)```'

    for match in re.finditer(pattern, content, re.DOTALL):
        code = match.group(2)
        line_num = content[:match.start()].count('\n') + 1

        # Check for TEST marker
        test_match = re.search(r'#\s*TEST:\s*(\w+)', code)
        if not test_match:
            continue  # Skip non-test blocks

        test_id = test_match.group(1)

        # Check for expected output
        expected_output = None
        output_match = re.search(r'#\s*EXPECTED_OUTPUT:\s*"([^"]*)"', code)
        if output_match:
            expected_output = output_match.group(1)

        # Remove test markers from code
        clean_code = re.sub(r'#\s*TEST:.*\n', '', code)
        clean_code = re.sub(r'#\s*EXPECTED_OUTPUT:.*\n', '', clean_code)

        blocks.append(
            ExecutableBlock(
                code=clean_code.strip(),
                file_path=markdown_path,
                line_num=line_num,
                test_id=test_id,
                expected_output=expected_output
            )
        )

    return blocks


def run_code_block(block: ExecutableBlock, timeout: int = 5) -> Tuple[bool, str, str]:
    """
    Execute a code block in an isolated Python subprocess.

    Returns:
        Tuple of (success, stdout, stderr)
    """
    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
        f.write(block.code)
        temp_file = Path(f.name)

    try:
        # Run code in subprocess
        result = subprocess.run(
            [sys.executable, str(temp_file)],
            capture_output=True,
            text=True,
            timeout=timeout
        )

        success = result.returncode == 0
        return success, result.stdout, result.stderr

    except subprocess.TimeoutExpired:
        return False, "", f"Execution timed out after {timeout}s"

    except Exception as e:
        return False, "", f"Execution error: {str(e)}"

    finally:
        # Clean up temp file
        temp_file.unlink(missing_ok=True)


def validate_output(actual_output: str, expected_output: Optional[str]) -> Tuple[bool, str]:
    """Validate actual output against expected output."""
    if expected_output is None:
        return True, "No expected output specified"

    actual_clean = actual_output.strip()
    expected_clean = expected_output.strip()

    if actual_clean == expected_clean:
        return True, "Output matches expected"
    else:
        return False, f"Expected: '{expected_clean}', Got: '{actual_clean}'"


def test_file(markdown_path: Path, verbose: bool = False) -> Tuple[int, int, List[str]]:
    """
    Test all executable blocks in a Markdown file.

    Returns:
        Tuple of (total_tests, failed_tests, error_messages)
    """
    blocks = extract_executable_blocks(markdown_path)

    if not blocks:
        if verbose:
            print(f"  No executable test blocks found in {markdown_path}")
        return 0, 0, []

    errors = []
    failed = 0

    for block in blocks:
        if verbose:
            print(f"  Running test: {block.test_id} ({block.file_path}:{block.line_num})")

        success, stdout, stderr = run_code_block(block)

        if not success:
            failed += 1
            error_msg = stderr if stderr else "Execution failed"
            errors.append(
                f"{block.file_path}:{block.line_num} - Test '{block.test_id}' FAILED: {error_msg}"
            )
            continue

        # Validate output if expected output is specified
        output_valid, msg = validate_output(stdout, block.expected_output)

        if not output_valid:
            failed += 1
            errors.append(
                f"{block.file_path}:{block.line_num} - Test '{block.test_id}' FAILED: {msg}"
            )
        elif verbose:
            print(f"    ✓ PASSED")

    return len(blocks), failed, errors


def main():
    """Main test routine."""
    import argparse

    parser = argparse.ArgumentParser(description='Test executable Python code blocks in Markdown')
    parser.add_argument('paths', nargs='*', help='Paths to Markdown files or directories')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--docs-dir', default='docs', help='Documentation directory (default: docs)')
    parser.add_argument('--timeout', type=int, default=5, help='Execution timeout in seconds')

    args = parser.parse_args()

    # Determine files to test
    files_to_test = []

    if args.paths:
        for path_str in args.paths:
            path = Path(path_str)
            if path.is_file():
                files_to_test.append(path)
            elif path.is_dir():
                files_to_test.extend(path.rglob('*.md'))
    else:
        # Default: test all markdown files in docs directory
        docs_path = Path(args.docs_dir)
        if docs_path.exists():
            files_to_test.extend(docs_path.rglob('*.md'))
        else:
            print(f"Error: Documentation directory '{args.docs_dir}' not found", file=sys.stderr)
            return 1

    if not files_to_test:
        print("No Markdown files found to test", file=sys.stderr)
        return 1

    # Run tests
    total_tests = 0
    total_failed = 0
    all_errors = []

    print(f"Testing executable code blocks in {len(files_to_test)} Markdown files...")

    for md_file in sorted(files_to_test):
        tests, failed, errors = test_file(md_file, verbose=args.verbose)
        total_tests += tests
        total_failed += failed
        all_errors.extend(errors)

    # Report results
    print(f"\n{'='*60}")
    print(f"Test Results:")
    print(f"  Files checked: {len(files_to_test)}")
    print(f"  Tests run: {total_tests}")
    print(f"  Tests passed: {total_tests - total_failed}")
    print(f"  Tests failed: {total_failed}")
    print(f"{'='*60}")

    if all_errors:
        print("\nFailed tests:")
        for error in all_errors:
            print(f"  {error}")
        return 1
    elif total_tests > 0:
        print("\n[PASS] All tests passed!")
        return 0
    else:
        print("\nNo executable tests found.")
        print("Mark code blocks for testing with '# TEST: test_name' comment.")
        return 0


if __name__ == '__main__':
    sys.exit(main())
