#!/usr/bin/env python3
"""
Generate a fleet config YAML for rmf_demos_fleet_adapter.

Creates a config file with N robots named robot_1..robot_N,
fleet name 'fleet_1', and identity reference coordinates (no transform).

Usage:
    python3 generate_fleet_config.py --num-robots 3 \
                                     --output /tmp/rmf_fleet_config.yaml
"""

import argparse
import yaml
import sys


def main():
    parser = argparse.ArgumentParser(
        description='Generate fleet config YAML for rmf_demos_fleet_adapter')
    parser.add_argument(
        '--num-robots', '-n', type=int, required=True,
        help='Number of robots')
    parser.add_argument(
        '--output', '-o', required=True,
        help='Output fleet config YAML path')
    parser.add_argument(
        '--fleet-name', default='fleet_1',
        help='Fleet name (default: fleet_1)')
    parser.add_argument(
        '--port', type=int, default=7001,
        help='Fleet manager REST API port (default: 7001)')
    args = parser.parse_args()

    robots = {}
    for i in range(1, args.num_robots + 1):
        robots[f'robot_{i}'] = {
            'charger': f'robot_{i}_charger',
        }

    config = {
        'rmf_fleet': {
            'name': args.fleet_name,
            'limits': {
                'linear': [2.0, 0.75],   # velocity (m/s), acceleration
                'angular': [1.5, 2.0],   # velocity (rad/s), acceleration
            },
            'profile': {
                'footprint': 0.3,  # radius in m
                'vicinity': 0.5,   # radius in m
            },
            'reversible': True,
            'battery_system': {
                'voltage': 12.0,
                'capacity': 24.0,
                'charging_current': 5.0,
            },
            'mechanical_system': {
                'mass': 20.0,
                'moment_of_inertia': 10.0,
                'friction_coefficient': 0.22,
            },
            'ambient_system': {
                'power': 20.0,
            },
            'tool_system': {
                'power': 0.0,
            },
            'recharge_threshold': 0.10,
            'recharge_soc': 1.0,
            'publish_fleet_state': 10.0,
            'account_for_battery_drain': True,
            'task_capabilities': {
                'loop': True,
                'delivery': True,
                'clean': False,
            },
            'finishing_request': 'park',
            'robots': robots,
        },
        'fleet_manager': {
            'ip': '127.0.0.1',
            'port': args.port,
            'user': 'some_user',
            'password': 'some_password',
            'reference_coordinates': {
                'rmf': [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]],
                'robot': [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]],
            },
        },
    }

    with open(args.output, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f'Fleet config written to {args.output}: '
          f'{args.num_robots} robots in fleet "{args.fleet_name}"')


if __name__ == '__main__':
    main()
