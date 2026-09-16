#!/usr/bin/env python3
# Copyright 2026, kibo. Apache-2.0
#
# compare_map: verify a saved SLAM map (PGM + YAML from nav2_map_server
# map_saver_cli) against the ground-truth SDF room dimensions.
#
# The ground truth (worlds/office_arena.sdf) is an enclosed room whose inner
# wall faces sit at x = +/-2 (width 4 m) and y = +/-4 (length 8 m). The lidar
# reflects off those inner faces, so the black perimeter of the map should span
# ~4 m x ~8 m at the map's resolution.
#
# Usage:
#   ros2 run tb3_office_sim compare_map --pgm maps/office_map.pgm \
#       --yaml maps/office_map.yaml                 # default 4x8 m office
#   ros2 run tb3_office_sim compare_map --pgm maps/maze_map.pgm \
#       --yaml maps/maze_map.yaml --size 5 5        # any arena
# Exit code 0 = PASS, 1 = FAIL, 2 = file error.

import argparse
import sys
from pathlib import Path

TOLERANCE_M = 0.2
OCCUPIED_THRESHOLD = 128  # pgm pixel < 128 => wall/obstacle

MAXVAL_SUPPORTED = 255


def parse_pgm(path: Path):
    """Parse P5 (binary) or P2 (ascii) Netpbm PGM. Returns (width, height, pixels)."""
    data = path.read_bytes()

    def next_token(i):
        # PGM supports comments (# ...) and arbitrary whitespace.
        while i < len(data):
            if data[i:i + 1] == b'#':
                while i < len(data) and data[i:i + 1] != b'\n':
                    i += 1
            elif data[i:i + 1] in b' \t\r\n':
                i += 1
            else:
                break
        j = i
        while j < len(data) and data[j:j + 1] not in b' \t\r\n#':
            j += 1
        return data[i:j].decode('ascii'), j

    magic, i = next_token(0)
    if magic not in ('P2', 'P5'):
        raise ValueError(f'unsupported PGM magic "{magic}" (expected P2 or P5)')
    width_s, i = next_token(i)
    height_s, i = next_token(i)
    maxval_s, i = next_token(i)
    width, height, maxval = int(width_s), int(height_s), int(maxval_s)
    if maxval > MAXVAL_SUPPORTED:
        raise ValueError('maxval > 255 not supported')

    pixels = []
    if magic == 'P5':
        # single whitespace char then width*height bytes
        while i < len(data) and data[i:i + 1] in b' \t\r\n':
            i += 1
        raw = data[i:i + width * height]
        if len(raw) < width * height:
            raise ValueError('truncated P5 pixel data')
        pixels = list(raw)
    else:  # P2 ascii
        for _ in range(width * height):
            tok, i = next_token(i)
            pixels.append(int(tok))
    return width, height, pixels


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pgm', required=True, help='path to map .pgm')
    parser.add_argument('--yaml', required=True, help='path to map .yaml')
    parser.add_argument(
        '--size', type=float, nargs=2, metavar=('W', 'H'), default=[4.0, 8.0],
        help='expected drivable arena size W x H in meters '
             '(default 4 8 = office arena)')
    args = parser.parse_args(argv)

    expected_w, expected_h = args.size[0], args.size[1]

    pgm_path = Path(args.pgm)
    yaml_path = Path(args.yaml)
    if not pgm_path.is_file() or not yaml_path.is_file():
        print(f'ERROR: files not found: {pgm_path} / {yaml_path}', file=sys.stderr)
        return 2

    # ---- read resolution from the map_server yaml ------------------------
    import yaml
    meta = yaml.safe_load(yaml_path.read_text())
    resolution = float(meta['resolution'])
    print(f'Map metadata: resolution={resolution} m/cell, '
          f'origin=({meta["origin"][0]}, {meta["origin"][1]})')

    # ---- measure occupied (wall) extents in the image --------------------
    width, height, pixels = parse_pgm(pgm_path)
    print(f'PGM: {width} x {height} px, threshold={OCCUPIED_THRESHOLD}')

    xs, ys = [], []
    for y in range(height):
        row = pixels[y * width:(y + 1) * width]
        for x in range(width):
            if row[x] < OCCUPIED_THRESHOLD:
                xs.append(x)
                ys.append(y)
    if not xs:
        print('FAIL: no occupied pixels found (empty map?)', file=sys.stderr)
        return 1

    # extent between the outermost occupied cell CENTERS, in meters
    extent_x = (max(xs) - min(xs)) * resolution
    extent_y = (max(ys) - min(ys)) * resolution
    print(f'Occupied pixel extent: {max(xs) - min(xs)} px x {max(ys) - min(ys)} px')
    print(f'Measured room extent:  {extent_x:.2f} m  x  {extent_y:.2f} m')

    # ---- compare against the arena ground truth ---------------------------
    dx = abs(extent_x - expected_w)
    dy = abs(extent_y - expected_h)
    ok = dx <= TOLERANCE_M and dy <= TOLERANCE_M
    print(f'Expected (arena inner faces): {expected_w} m x {expected_h} m')
    print(f'Delta: {dx:.2f} m (x)  {dy:.2f} m (y)   tolerance +/-{TOLERANCE_M} m')
    label = f'{expected_w:.0f} x {expected_h:.0f} m arena'
    print('PASS: SLAM map scale matches the ' + label if ok
          else 'FAIL: map does NOT match the ' + label + ' ground truth')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())