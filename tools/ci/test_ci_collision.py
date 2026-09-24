#!/usr/bin/env python3
"""Stationary collision baseline; wall or missing telemetry must fail this test."""
import argparse
import json
from pathlib import Path
import time
import os
import xml.etree.ElementTree as ET

import rclpy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('ci_collision_validation')
    state = {'last': None, 'session': None, 'samples': 0, 'failure': None}

    def receive(message):
        try:
            value = json.loads(message.data)
            if value.get('configured') is not True or not value.get('session'):
                raise ValueError('Collision publisher not configured')
            if type(value.get('count')) is not int or type(value.get('active')) is not int:
                raise ValueError('Invalid collision counters')
            if value['count'] < 0 or value['active'] < 0:
                raise ValueError('Negative collision counters')
            if value['count'] or value['active']:
                raise ValueError('Obstacle contact: ' + value.get('last_object', 'unknown'))
            if state['session'] is not None and state['session'] != value['session']:
                raise ValueError('Collision publisher restarted')
            state.update(last=time.monotonic(), session=value['session'], samples=state['samples'] + 1)
        except (ValueError, TypeError, KeyError) as error:
            state['failure'] = str(error)

    node.create_subscription(String, '/test/collision_status', receive, 100)
    started = time.monotonic()
    try:
        while time.monotonic() - started < 30:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            process_failure = os.environ.get('CI_PROCESS_FAILURE_FILE')
            if process_failure and Path(process_failure).exists():
                raise RuntimeError(Path(process_failure).read_text())
            if state['failure']:
                raise RuntimeError(state['failure'])
            if state['last'] is None and now - started > 10:
                raise RuntimeError('Missing collision telemetry')
            if state['last'] is not None and now - state['last'] > 1:
                raise RuntimeError('Collision heartbeat stale')
    except (Exception, KeyboardInterrupt) as error:
        state['failure'] = str(error) or type(error).__name__
    finally:
        node.destroy_node()
        rclpy.shutdown()
    passed = state['failure'] is None and state['samples'] > 0
    report = dict(passed=passed, **state)
    (output / 'collision-results.json').write_text(json.dumps(report, indent=2))
    suite = ET.Element('testsuite', name='collision_validation', tests='1', failures=str(int(not passed)))
    case = ET.SubElement(suite, 'testcase', name='no_obstacle_contact_and_live_telemetry')
    if not passed:
        ET.SubElement(case, 'failure', message=state['failure'] or 'No samples')
        print('::error title=Collision validation::' + (state['failure'] or 'No samples'), flush=True)
    ET.ElementTree(suite).write(output / 'junit.xml', encoding='utf-8')
    print(json.dumps(report), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
