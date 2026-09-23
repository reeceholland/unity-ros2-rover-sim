#!/usr/bin/env python3
"""Run map-frame Nav2 goals and bounded fault windows; emit JSON and JUnit."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

from navigation_scenario import load_scenario, pose_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--feedback-interval', type=float, default=1.0,
                        help='Seconds between verbose feedback lines; 0 prints every feedback message')
    args = parser.parse_args()
    if not math.isfinite(args.feedback_interval) or args.feedback_interval < 0:
        parser.error('--feedback-interval must be finite and nonnegative')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    events = []
    node = None
    handle = None
    active = set()
    started = time.monotonic()
    failure = None

    def log(message):
        print(f'[{time.monotonic() - started:8.2f}s] {message}', flush=True)

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        config = load_scenario(args.scenario)
        log(f"Loaded {len(config['goals'])} map-frame goals from {args.scenario}")
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.time import Time
        from rclpy.qos import qos_profile_sensor_data
        from lifecycle_msgs.srv import GetState
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener, TransformException
        from ros2_fault_injection.srv import SetFaultState

        rclpy.init()
        node = rclpy.create_node('ci_navigation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        service = node.create_client(SetFaultState, '/fault_injection/set_fault_state')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        state = {'sim': None, 'advance': time.monotonic(), 'reset': False, 'scan': 0}

        def clock(message):
            value = message.clock.sec + message.clock.nanosec * 1e-9
            old = state['sim']
            if old is not None and value < old:
                state['reset'] = True
            if old is None or value > old:
                state['advance'] = time.monotonic()
            state['sim'] = value

        def scan(_):
            state['scan'] = time.monotonic()

        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan_raw', scan, qos_profile_sensor_data)

        def spin(check_clock=True):
            rclpy.spin_once(node, timeout_sec=0.02)
            if check_clock and (state['reset'] or time.monotonic() - state['advance'] > config['clock_timeout']):
                raise RuntimeError('Simulation clock reset or stopped advancing')

        def wait(future, seconds=5, check_clock=True):
            deadline = time.monotonic() + seconds
            while not future.done() and time.monotonic() < deadline:
                spin(check_clock)
            if not future.done():
                raise RuntimeError('ROS service/action response timed out')
            return future.result()

        def fault_state(fault_id, enabled, cleanup=False):
            # Track attempted activation too: a timed-out request may still take effect.
            if enabled:
                active.add(fault_id)
            response = wait(service.call_async(SetFaultState.Request(fault_id=fault_id, active=enabled)), check_clock=not cleanup)
            if not response.success:
                raise RuntimeError(response.message)
            if not enabled:
                active.discard(fault_id)
            log(f"FAULT {fault_id}: {'ENABLED' if enabled else 'DISABLED'}; sim={state['sim']:.3f}s")
            events.append({'id': fault_id, 'active': enabled, 'wall_elapsed': time.monotonic() - started, 'sim_time': state['sim']})

        def pose():
            transform = buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if state['sim'] is None or abs(state['sim'] - stamp) > 2:
                raise RuntimeError('Map pose transform is stale')
            return transform.transform

        log('Waiting for clock, scan, map TF, Nav2 and fault service')
        deadline = time.monotonic() + config['ready_timeout']
        while True:
            spin(False)
            try:
                pose()
                ready = (state['sim'] is not None and time.monotonic() - state['advance'] < 1
                         and time.monotonic() - state['scan'] < 1
                         and client.server_is_ready() and service.service_is_ready())
                if ready:
                    break
            except (TransformException, RuntimeError):
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError('Readiness timeout: need clock, raw scan, map TF, Nav2 action and fault service')
        # Action discovery precedes lifecycle activation; wait for all servers.
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            log(f'Waiting for {name} lifecycle activation')
            lifecycle = node.create_client(GetState, f'/{name}/get_state')
            while True:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Readiness timeout: {name} is not active')
                if lifecycle.service_is_ready():
                    response = wait(lifecycle.call_async(GetState.Request()), seconds=min(5, max(0.01, deadline - time.monotonic())))
                    if response.current_state.id == 3:
                        break
                for _ in range(10):
                    spin()
        fault_state('ci_scan_dropout', False)
        for index, goal in enumerate(config['goals'], 1):
            record = {'name': goal['name'], 'passed': False}
            results.append(record)
            goal_start = time.monotonic()
            message = NavigateToPose.Goal()
            message.pose.header.frame_id = 'map'
            message.pose.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose.position.x = float(goal['x'])
            message.pose.pose.position.y = float(goal['y'])
            yaw = math.radians(goal['yaw_deg'])
            message.pose.pose.orientation.z = math.sin(yaw / 2)
            message.pose.pose.orientation.w = math.cos(yaw / 2)
            log(f"GOAL {index}/{len(config['goals'])} {goal['name']}: x={goal['x']:.6f} y={goal['y']:.6f} yaw={goal['yaw_deg']:.6f}deg; deadline={goal['timeout']}s; faults={goal.get('faults', [])}")
            last_feedback = -math.inf

            def feedback(message):
                nonlocal last_feedback
                now = time.monotonic()
                if now - last_feedback < args.feedback_interval:
                    return
                last_feedback = now
                value = message.feedback
                position = value.current_pose.pose.position
                orientation = value.current_pose.pose.orientation
                navigation_time = value.navigation_time.sec + value.navigation_time.nanosec * 1e-9
                eta = value.estimated_time_remaining.sec + value.estimated_time_remaining.nanosec * 1e-9
                log(f"FEEDBACK {goal['name']}: frame={value.current_pose.header.frame_id} "
                    f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}) "
                    f"quaternion=({orientation.x:.6f}, {orientation.y:.6f}, {orientation.z:.6f}, {orientation.w:.6f}) "
                    f"remaining={value.distance_remaining:.3f}m navigation_time={navigation_time:.2f}s "
                    f"ETA={eta:.2f}s recoveries={value.number_of_recoveries}")

            handle = wait(client.send_goal_async(message, feedback_callback=feedback))
            if not handle.accepted:
                raise RuntimeError(f"Goal rejected: {goal['name']}")
            log(f"ACCEPTED {goal['name']}")
            pending = handle.get_result_async()
            schedule_start = time.monotonic()
            enabled = set()
            disabled = set()
            while not pending.done():
                spin()
                elapsed = time.monotonic() - schedule_start
                if time.monotonic() - goal_start > goal['timeout']:
                    raise RuntimeError(f"Goal deadline exceeded: {goal['name']}")
                for fault in goal.get('faults', []):
                    fid = fault['id']
                    if elapsed >= fault['start'] and fid not in enabled:
                        fault_state(fid, True)
                        enabled.add(fid)
                    if elapsed >= fault['start'] + fault['duration'] and fid not in disabled:
                        fault_state(fid, False)
                        disabled.add(fid)
            result = pending.result()
            handle = None
            log(f"RESULT {goal['name']}: status={result.status} (4=SUCCEEDED), error_code={getattr(result.result, 'error_code', None)}, error_msg={getattr(result.result, 'error_msg', '')!r}")
            record.update(status=result.status, error_code=getattr(result.result, 'error_code', None), duration=time.monotonic() - goal_start)
            if result.status != GoalStatus.STATUS_SUCCEEDED or getattr(result.result, 'error_code', 0) != 0:
                raise RuntimeError(f"Navigation failed: {record}; {getattr(result.result, 'error_msg', '')}")
            if len(disabled) != len(goal.get('faults', [])):
                raise RuntimeError('Goal completed before all fault windows ran; increase waypoint distance or shorten fault windows')
            current = pose()
            q = current.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            distance, angle = pose_errors(goal, current.translation.x, current.translation.y, yaw)
            record.update(position_error=distance, yaw_error_deg=angle)
            if not math.isfinite(distance + angle) or distance > goal['position_tolerance'] or angle > goal['yaw_tolerance_deg']:
                raise RuntimeError(f'Final pose outside tolerance: {record}')
            record['passed'] = True
            log(f"PASS {goal['name']}: position_error={distance:.3f}m yaw_error={angle:.3f}deg duration={record['duration']:.2f}s")
    except (Exception, KeyboardInterrupt) as error:
        failure = f'{type(error).__name__}: {error}'
        print(f'FAIL: {failure}', flush=True)
    finally:
        if node is not None:
            cleanup_errors = []
            if handle is not None:
                try:
                    wait(handle.cancel_goal_async(), check_clock=False)
                except Exception as error:
                    cleanup_errors.append(str(error))
            for fid in list(active):
                try:
                    fault_state(fid, False, cleanup=True)
                except Exception as error:
                    cleanup_errors.append(str(error))
            if cleanup_errors:
                failure = (failure or '') + '; cleanup failed: ' + '; '.join(cleanup_errors)
            node.destroy_node()
            rclpy.shutdown()
        if failure:
            if not results or results[-1]['passed']:
                results.append({'name': 'setup_or_cleanup', 'passed': False})
            results[-1]['error'] = failure
        report = {'passed': failure is None, 'elapsed': time.monotonic() - started, 'goals': results, 'fault_events': events}
        (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
        suite = ET.Element('testsuite', name='rover_navigation', tests=str(len(results)), failures=str(sum(not r['passed'] for r in results)))
        for record in results:
            case = ET.SubElement(suite, 'testcase', name=record['name'], time=str(record.get('duration', 0)))
            if not record['passed']:
                ET.SubElement(case, 'failure', message=record.get('error', 'failed'))
        ET.ElementTree(suite).write(output / 'junit.xml', encoding='utf-8', xml_declaration=True)
    log(f"FINISHED: {'PASS' if failure is None else 'FAIL'}; reports: {output / 'results.json'}, {output / 'junit.xml'}")
    return 0 if failure is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
