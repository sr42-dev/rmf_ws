#!/usr/bin/env python3
"""
Generate an RMF nav graph YAML from a building.yaml file.

Reads the building map vertices (pixel coords), converts to meters
using PIXEL_TO_METER = 1/30, and writes the nav graph YAML format
expected by rmf_fleet_adapter's parse_graph().

Usage:
    python3 generate_nav_graph.py --building <path/to/map.building.yaml> \
                                  --output <path/to/nav_graph.yaml>
"""

import argparse
import yaml
import sys


PIXEL_TO_METER = 1.0 / 30.0


def main():
    parser = argparse.ArgumentParser(
        description='Generate RMF nav graph from building YAML')
    parser.add_argument(
        '--building', '-b', required=True,
        help='Path to map.building.yaml')
    parser.add_argument(
        '--output', '-o', required=True,
        help='Output nav graph YAML path')
    args = parser.parse_args()

    with open(args.building, 'r') as f:
        building = yaml.safe_load(f)

    nav_graph = {'levels': {}}

    for level_name, level_data in building.get('levels', {}).items():
        vertices_out = []
        for v in level_data.get('vertices', []):
            # Building YAML vertex format: [x_px, y_px, flags, name_str]
            x_m = v[0] * PIXEL_TO_METER
            y_m = v[1] * PIXEL_TO_METER
            options = {}
            # v[3] is the vertex name (may be empty string)
            if len(v) > 3 and v[3]:
                options['name'] = v[3]
            vertices_out.append([round(x_m, 6), round(y_m, 6), options])

        lanes_out = []
        for lane in level_data.get('lanes', []):
            # Building YAML lane format: [start_idx, end_idx, {props}]
            if len(lane) >= 2:
                lane_options = {}
                # Propagate bidirectional flag from building YAML
                # Property format: {bidirectional: [type_id, value]}
                if len(lane) > 2 and isinstance(lane[2], dict):
                    bidir_prop = lane[2].get('bidirectional', [4, True])
                    if isinstance(bidir_prop, list) and len(bidir_prop) >= 2:
                        lane_options['bidirectional'] = bool(bidir_prop[1])
                    elif isinstance(bidir_prop, bool):
                        lane_options['bidirectional'] = bidir_prop
                    else:
                        lane_options['bidirectional'] = True
                else:
                    lane_options['bidirectional'] = True
                lanes_out.append([lane[0], lane[1], lane_options])

        nav_graph['levels'][level_name] = {
            'vertices': vertices_out,
            'lanes': lanes_out,
        }

    with open(args.output, 'w') as f:
        yaml.dump(nav_graph, f, default_flow_style=None, sort_keys=False)

    n_verts = sum(
        len(l['vertices']) for l in nav_graph['levels'].values())
    n_lanes = sum(
        len(l['lanes']) for l in nav_graph['levels'].values())
    print(f'Nav graph written to {args.output}: '
          f'{n_verts} vertices, {n_lanes} lanes')


if __name__ == '__main__':
    main()
