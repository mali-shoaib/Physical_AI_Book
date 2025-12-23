#!/usr/bin/env python3
"""
Validation script for URDF/SDF/XML code blocks in Markdown files.

This script:
1. Extracts URDF/SDF/XML code blocks from Markdown files
2. Validates XML syntax
3. Checks URDF-specific structure (links, joints, robot tags)
4. Checks SDF-specific structure (models, worlds, version attributes)
5. Reports errors with line numbers and file locations
"""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Tuple


class URDFBlock:
    """Represents a URDF/XML code block extracted from Markdown."""

    def __init__(self, code: str, language: str, file_path: Path, line_num: int):
        self.code = code
        self.language = language
        self.file_path = file_path
        self.line_num = line_num


def extract_urdf_blocks(markdown_path: Path) -> List[URDFBlock]:
    """Extract all URDF/XML/SDF code blocks from a Markdown file."""
    urdf_blocks = []

    with open(markdown_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern matches fenced code blocks
    pattern = r'```(\w+)?\n(.*?)```'

    for match in re.finditer(pattern, content, re.DOTALL):
        language = match.group(1) or 'text'
        code = match.group(2)

        # Only process XML, URDF, SDF, or markup blocks
        if language.lower() in ('xml', 'urdf', 'sdf', 'markup'):
            # Calculate line number of code block start
            line_num = content[:match.start()].count('\n') + 1

            urdf_blocks.append(URDFBlock(code, language, markdown_path, line_num))

    return urdf_blocks


def validate_xml_syntax(urdf_block: URDFBlock) -> Tuple[bool, str]:
    """Validate XML syntax."""
    try:
        ET.fromstring(urdf_block.code)
        return True, ""
    except ET.ParseError as e:
        return False, f"XML parse error: {e.msg} (line {e.position[0]})"
    except Exception as e:
        return False, f"Parse error: {str(e)}"


def check_sdf_structure(urdf_block: URDFBlock) -> List[str]:
    """Check SDF-specific structure and best practices."""
    warnings = []

    try:
        root = ET.fromstring(urdf_block.code)

        # Check for sdf root element
        if root.tag != 'sdf':
            warnings.append("Root element should be <sdf> for SDF files, found <{}>".format(root.tag))
            return warnings

        # Check SDF has version attribute
        if 'version' not in root.attrib:
            warnings.append("SDF element missing 'version' attribute")

        # Check for model or world element
        model = root.find('model')
        world = root.find('world')

        if not model and not world:
            warnings.append("SDF should contain either <model> or <world> element")

        # If model exists, check structure
        if model is not None:
            if 'name' not in model.attrib:
                warnings.append("Model element missing 'name' attribute")

            # Check for at least one link
            links = model.findall('link')
            if not links:
                warnings.append("Model should have at least one <link> element")

            # Check link names
            link_names = set()
            for link in links:
                if 'name' not in link.attrib:
                    warnings.append("Link element missing 'name' attribute")
                else:
                    name = link.attrib['name']
                    if name in link_names:
                        warnings.append(f"Duplicate link name: '{name}'")
                    link_names.add(name)

        # If world exists, check structure
        if world is not None:
            if 'name' not in world.attrib:
                warnings.append("World element missing 'name' attribute")

    except ET.ParseError:
        # Already caught in validate_xml_syntax
        pass
    except Exception as e:
        warnings.append(f"SDF structure validation error: {str(e)}")

    return warnings


def check_urdf_structure(urdf_block: URDFBlock) -> List[str]:
    """Check URDF-specific structure and best practices."""
    warnings = []

    try:
        root = ET.fromstring(urdf_block.code)

        # Check for robot root element
        if root.tag != 'robot':
            warnings.append("Root element should be <robot>, found <{}>".format(root.tag))
            return warnings

        # Check robot has a name attribute
        if 'name' not in root.attrib:
            warnings.append("Robot element missing 'name' attribute")

        # Check for at least one link
        links = root.findall('link')
        if not links:
            warnings.append("No <link> elements found (URDF should have at least one link)")

        # Check for link names
        link_names = set()
        for link in links:
            if 'name' not in link.attrib:
                warnings.append("Link element missing 'name' attribute")
            else:
                name = link.attrib['name']
                if name in link_names:
                    warnings.append(f"Duplicate link name: '{name}'")
                link_names.add(name)

        # Check joints reference valid links
        joints = root.findall('joint')
        joint_names = set()

        for joint in joints:
            if 'name' not in joint.attrib:
                warnings.append("Joint element missing 'name' attribute")
            else:
                name = joint.attrib['name']
                if name in joint_names:
                    warnings.append(f"Duplicate joint name: '{name}'")
                joint_names.add(name)

            if 'type' not in joint.attrib:
                warnings.append(f"Joint missing 'type' attribute")

            # Check parent and child links exist
            parent = joint.find('parent')
            child = joint.find('child')

            if parent is not None and 'link' in parent.attrib:
                parent_link = parent.attrib['link']
                if parent_link not in link_names:
                    warnings.append(f"Joint references non-existent parent link: '{parent_link}'")

            if child is not None and 'link' in child.attrib:
                child_link = child.attrib['link']
                if child_link not in link_names:
                    warnings.append(f"Joint references non-existent child link: '{child_link}'")

        # Check for common issues
        if links and not joints:
            warnings.append("URDF has links but no joints (consider adding joints to connect links)")

    except ET.ParseError:
        # Already caught in validate_xml_syntax
        pass
    except Exception as e:
        warnings.append(f"Structure validation error: {str(e)}")

    return warnings


def validate_file(markdown_path: Path, verbose: bool = False) -> Tuple[int, int, List[str]]:
    """
    Validate all URDF/XML code blocks in a Markdown file.

    Returns:
        Tuple of (total_blocks, error_count, error_messages)
    """
    urdf_blocks = extract_urdf_blocks(markdown_path)

    if not urdf_blocks:
        if verbose:
            print(f"  No URDF/XML code blocks found in {markdown_path}")
        return 0, 0, []

    errors = []
    error_count = 0

    for block in urdf_blocks:
        # Validate XML syntax
        is_valid, error_msg = validate_xml_syntax(block)

        if not is_valid:
            error_count += 1
            errors.append(
                f"{block.file_path}:{block.line_num} - {error_msg}"
            )
            continue  # Skip structure check if XML is invalid

        # Check structure based on language
        if block.language.lower() == 'sdf':
            warnings = check_sdf_structure(block)
        else:
            warnings = check_urdf_structure(block)

        if warnings:
            for warning in warnings:
                errors.append(
                    f"{block.file_path}:{block.line_num} - Warning: {warning}"
                )

    if verbose:
        print(f"  Validated {len(urdf_blocks)} URDF/XML blocks in {markdown_path}")

    return len(urdf_blocks), error_count, errors


def main():
    """Main validation routine."""
    import argparse

    parser = argparse.ArgumentParser(description='Validate URDF/XML code in Markdown files')
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

    print(f"Validating {len(files_to_validate)} Markdown files for URDF/XML...")

    for md_file in sorted(files_to_validate):
        blocks, errors, messages = validate_file(md_file, verbose=args.verbose)
        total_blocks += blocks
        total_errors += errors
        all_errors.extend(messages)

    # Report results
    print(f"\n{'='*60}")
    print(f"URDF Validation Results:")
    print(f"  Files checked: {len(files_to_validate)}")
    print(f"  URDF/XML blocks validated: {total_blocks}")
    print(f"  Errors found: {total_errors}")
    print(f"{'='*60}")

    if all_errors:
        print("\nErrors and warnings:")
        for error in all_errors:
            print(f"  {error}")
        return 1
    else:
        print("\n[PASS] All URDF/XML code is valid!")
        return 0


if __name__ == '__main__':
    sys.exit(main())
