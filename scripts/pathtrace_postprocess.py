#!/usr/bin/env python3
"""
Pathtracer Post-processing Script

Reads pathtracer output from a file and generates a PFM (Portable Float Map) file.
Optionally skips a specified substring at the beginning of each input line.

Expected input format:
PIXEL_BUFFER_START
width height
(R,G,B) (R,G,B) ... (width RGB triplets per line)
... (height lines)
PIXEL_BUFFER_END

Usage:
python3 pathtrace_postprocess.py <input file> <output file> [skip_substring]

Arguments:
    input.txt: Input filename containing pathtracer output
    output.pfm: Output filename for the PFM file
    skip_substring: Optional substring to skip at the beginning of each line
"""

import sys
import struct
import re


def write_pfm(filename, image, width, height):
    """
    Write a PFM (Portable Float Map) file.
    
    Args:
        filename: Output filename
        image: List of RGB float triplets
        width: Image width
        height: Image height
    """
    with open(filename, 'wb') as f:
        # PFM header
        f.write(b'PF\n')  # Color PFM magic number
        f.write(f'{width} {height}\n'.encode('ascii'))
        f.write(b'-1.0\n')  # Little-endian scale factor
        
        # Write pixel data (bottom-to-top for PFM format)
        for y in range(height - 1, -1, -1):
            for x in range(width):
                idx = y * width + x
                r, g, b = image[idx]
                # Write as little-endian floats
                f.write(struct.pack('<f', b))
                f.write(struct.pack('<f', g))
                f.write(struct.pack('<f', r))


def parse_rgb_triplet(triplet_str):
    """
    Parse a single RGB triplet string like "(1.0, 0.5, 0.2)"
    
    Returns:
        tuple: (r, g, b) as floats
    """
    # Remove parentheses and split by comma
    clean_str = triplet_str.strip('()')
    parts = clean_str.split(',')
    
    if len(parts) != 3:
        raise ValueError(f"Invalid RGB triplet: {triplet_str}")
    
    try:
        r = float(parts[0].strip())
        g = float(parts[1].strip())
        b = float(parts[2].strip())
        return (r, g, b)
    except ValueError as e:
        raise ValueError(f"Could not parse RGB values from {triplet_str}: {e}")


def main():
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Supplied " + str(len(sys.argv) - 1) + " arguments, expected 2 or 3.", file=sys.stderr)
        print("Usage: python3 pathtrace_postprocess.py <input.txt> <output.pfm> [skip_substring]", file=sys.stderr)
        sys.exit(1)
    
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    skip_substring = sys.argv[3] if len(sys.argv) == 4 else None
    
    # Read input from file
    lines = []
    try:
        with open(input_filename, 'r') as f:
            for line in f:
                line_content = line.strip()
                # Skip the specified substring at the beginning of each line if provided
                if skip_substring and line_content.startswith(skip_substring):
                    line_content = line_content[len(skip_substring):]
                lines.append(line_content)
    except FileNotFoundError:
        print(f"Error: Could not find input file '{input_filename}'", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading input file: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Find the pixel buffer section
    try:
        start_idx = lines.index("PIXEL_BUFFER_START")
    except ValueError:
        print("Error: Could not find PIXEL_BUFFER_START marker", file=sys.stderr)
        sys.exit(1)
    
    try:
        end_idx = lines.index("PIXEL_BUFFER_END")
    except ValueError:
        print("Error: Could not find PIXEL_BUFFER_END marker", file=sys.stderr)
        sys.exit(1)
    
    if end_idx <= start_idx + 1:
        print("Error: Invalid pixel buffer section", file=sys.stderr)
        sys.exit(1)
    
    # Parse dimensions
    try:
        dimensions_line = lines[start_idx + 1]
        width, height = map(int, dimensions_line.split(', '))
    except (ValueError, IndexError):
        print(f"Error: Could not parse dimensions from line: {lines[start_idx + 1]}", file=sys.stderr)
        sys.exit(1)
    
    print(f"Parsing image with dimensions: {width}x{height}")
    
    # Parse pixel data
    pixel_lines = lines[start_idx + 2:end_idx]
    
    if len(pixel_lines) != width:
        print(f"Error: Expected {width} pixel lines, but got {len(pixel_lines)}", file=sys.stderr)
        sys.exit(1)
    
    image = []
    
    # Regular expression to match RGB triplets
    rgb_pattern = r'\(([^)]+)\)'
    
    for i, line in enumerate(pixel_lines):
        # Find all RGB triplets in the line
        matches = re.findall(rgb_pattern, line)
        
        if len(matches) != height:
            print(f"Error: Line {i} has {len(matches)} RGB triplets, expected {height}", file=sys.stderr)
            sys.exit(1)
        
        for match in matches:
            try:
                r, g, b = parse_rgb_triplet(f"({match})")
                image.append((r, g, b))
            except ValueError as e:
                print(f"Error parsing RGB triplet on line {i}: {e}", file=sys.stderr)
                sys.exit(1)
    
    if len(image) != width * height:
        print(f"Error: Expected {width * height} pixels, but got {len(image)}", file=sys.stderr)
        sys.exit(1)
    
    # Write PFM file
    try:
        write_pfm(output_filename, image, width, height)
        print(f"Successfully wrote PFM file: {output_filename}")
    except Exception as e:
        print(f"Error writing PFM file: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()