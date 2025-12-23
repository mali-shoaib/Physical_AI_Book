#!/usr/bin/env python3
"""
Master validation script that runs all validation checks.

This script orchestrates:
1. Python code syntax validation (validate_examples.py)
2. URDF/XML validation (validate_urdf.py)
3. Import checking (validate_imports.py)
4. Executable test running (test_code_examples.py)

Returns non-zero exit code if any validation fails.
"""

import sys
import subprocess
from pathlib import Path
from typing import List, Tuple


def run_validator(
    script_name: str,
    description: str,
    args: List[str] = None,
    verbose: bool = False
) -> Tuple[bool, str]:
    """
    Run a validation script and capture results.

    Returns:
        Tuple of (success, output_message)
    """
    script_path = Path(__file__).parent / script_name

    if not script_path.exists():
        return False, f"Validator script not found: {script_path}"

    cmd = [sys.executable, str(script_path)]

    if args:
        cmd.extend(args)

    if verbose:
        cmd.append('-v')

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300  # 5 minute timeout
        )

        success = result.returncode == 0

        # Combine stdout and stderr
        output = result.stdout
        if result.stderr:
            output += "\n" + result.stderr

        return success, output

    except subprocess.TimeoutExpired:
        return False, f"{description} timed out after 5 minutes"

    except Exception as e:
        return False, f"Error running {description}: {str(e)}"


def print_section_header(title: str):
    """Print a formatted section header."""
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}\n")


def main():
    """Run all validation checks."""
    import argparse

    parser = argparse.ArgumentParser(description='Run all validation checks')
    parser.add_argument('paths', nargs='*', help='Paths to validate (default: docs/)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--skip-tests', action='store_true', help='Skip executable tests')
    parser.add_argument('--skip-imports', action='store_true', help='Skip import validation')
    parser.add_argument('--docs-dir', default='docs', help='Documentation directory')

    args = parser.parse_args()

    print("=" * 70)
    print("  RUNNING ALL VALIDATION CHECKS")
    print("=" * 70)

    # Prepare arguments for validators
    validator_args = args.paths if args.paths else ['--docs-dir', args.docs_dir]

    results = []
    all_passed = True

    # 1. Python code syntax validation
    print_section_header("1. Python Code Syntax Validation")

    success, output = run_validator(
        'validate_examples.py',
        'Python syntax validation',
        validator_args,
        args.verbose
    )

    results.append(('Python Syntax', success))

    if args.verbose or not success:
        print(output)
    else:
        # Print summary line only
        lines = output.strip().split('\n')
        for line in lines:
            if 'Validation Results:' in line or line.startswith('  ') or '✓' in line:
                print(line)

    if not success:
        all_passed = False

    # 2. URDF/XML validation
    print_section_header("2. URDF/XML Validation")

    success, output = run_validator(
        'validate_urdf.py',
        'URDF/XML validation',
        validator_args,
        args.verbose
    )

    results.append(('URDF/XML', success))

    if args.verbose or not success:
        print(output)
    else:
        lines = output.strip().split('\n')
        for line in lines:
            if 'Validation Results:' in line or line.startswith('  ') or '✓' in line:
                print(line)

    if not success:
        all_passed = False

    # 3. Import validation (optional)
    if not args.skip_imports:
        print_section_header("3. Import Validation")

        import_args = validator_args + ['--show-ros2']

        success, output = run_validator(
            'validate_imports.py',
            'Import validation',
            import_args,
            args.verbose
        )

        results.append(('Imports', success))

        if args.verbose or not success:
            print(output)
        else:
            lines = output.strip().split('\n')
            for line in lines:
                if 'Validation Results:' in line or line.startswith('  ') or '✓' in line or 'ROS 2' in line:
                    print(line)

        # Import validation failures are warnings only (don't fail build)
        # unless ROS 2 packages are actually broken

    # 4. Executable tests (optional)
    if not args.skip_tests:
        print_section_header("4. Executable Code Tests")

        success, output = run_validator(
            'test_code_examples.py',
            'Executable tests',
            validator_args,
            args.verbose
        )

        results.append(('Executable Tests', success))

        if args.verbose or not success:
            print(output)
        else:
            lines = output.strip().split('\n')
            for line in lines:
                if 'Test Results:' in line or line.startswith('  ') or '✓' in line or 'No executable' in line:
                    print(line)

        if not success:
            all_passed = False

    # Final summary
    print("\n" + "=" * 70)
    print("  VALIDATION SUMMARY")
    print("=" * 70)

    for check_name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {check_name:25} {status}")

    print("=" * 70)

    if all_passed:
        print("\n[PASS] ALL VALIDATION CHECKS PASSED!\n")
        return 0
    else:
        print("\n[FAIL] SOME VALIDATION CHECKS FAILED\n")
        return 1


if __name__ == '__main__':
    sys.exit(main())
