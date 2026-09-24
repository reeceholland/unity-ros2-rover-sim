#!/usr/bin/env python3
"""Run map-frame Nav2 goals and bounded fault windows; emit JSON and JUnit."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

from navigation_scenario import load_scenario, pose_errors
from resilience_observer import Observer, Limits, write_report
from controller_readiness import wait_for_drive_controller


def navigation_summary(report):
    lines = ['### Headless navigation diagnostics', '',
             '**' + ('PASS' if report['passed'] else 'FAIL') + '**', '']
    missing = [name for name, ready in report.get('readiness', {}).items() if not ready]
    if missing:
        lines += ['Missing/stale readiness: ' + ', '.join(missing), '']
    for goal in report['goals']:
        lines.append(f"- {goal['name']}: {'PASS' if goal['passed'] else 'FAIL'}")
        for number, attempt in enumerate(goal.get('attempts', []), 1):
            lines.append(f"  - Attempt {number}: status={attempt['status']}, error_code={attempt['error_code']}")
        if goal.get('recovery_policy'):
            lines.append('  - Recovery: one explicit retry after healthy scans and fresh TF.')
        if goal.get('error'):
            lines.append('  - ' + str(goal['error']).replace('\n', ' '))
    for name, detail in report['observer'].get('failures', {}).items():
        lines.append(f'- Observer `{name}`: {detail}')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--observer-config', default=str(Path(__file__).with_name('observer_limits.json')))
    parser.add_argument('--dropout-duration', type=float, help='Override scheduled dropout duration in wall seconds')
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
    observer = None
    observer_report = None
    printed_failures = set()
    printed_events = 0
    readiness = {}

    def log(message):
        print(f'[{time.monotonic() - started:8.2f}s] {message}', flush=True)

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        config = load_scenario(args.scenario)
        observer = Observer(Limits(**json.loads(Path(args.observer_config).read_text())))
        if args.dropout_duration is not None:
            if not math.isfinite(args.dropout_duration) or args.dropout_duration <= 0:
                raise ValueError('--dropout-duration must be finite and positive')
            for goal in config['goals']:
                for fault in goal.get('faults', []):
                    if fault['start'] + args.dropout_duration >= goal['timeout']:
                        raise ValueError('Overridden dropout must fit within the goal deadline')
                    fault['duration'] = args.dropout_duration
        (output / 'effective_scenario.json').write_text(json.dumps(config, indent=2) + '\n')
        if sum(len(g.get('faults', [])) for g in config['goals']) != 1:
            raise ValueError('Resilience observer requires exactly one dropout per scenario')
        log(f"Loaded {len(config['goals'])} map-frame goals from {args.scenario}")
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.time import Time
        from rclpy.qos import qos_profile_sensor_data
        from lifecycle_msgs.srv import GetState
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, JointState
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
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

        def sim_time():
            return node.get_clock().now().nanoseconds * 1e-9

        def observed_scan(message):
            if not message.ranges:
                observer.fail('scan_valid', 'Empty injected scan')
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            observer.scan(sim_time(), stamp)

        def ground_truth(message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if abs(sim_time() - stamp) > observer.limits.telemetry_timeout:
                observer.fail('motion_timestamp', 'Stale Unity ground truth')
                return
            p, v = message.pose.pose.position, message.twist.twist
            observer.motion(sim_time(), p.x, p.y, math.hypot(v.linear.x, v.linear.y), v.angular.z)

        def motor_command(message):
            required = {'front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint'}
            values = dict(zip(message.name, message.velocity))
            if not required <= values.keys() or not all(math.isfinite(values[k]) for k in required):
                observer.fail('command_valid', 'Missing/non-finite wheel velocity command')
                return
            # Actual actuator input: max absolute wheel speed, radians/second.
            observer.velocity(sim_time(), max(abs(values[k]) for k in required), 0.0)

        node.create_subscription(LaserScan, '/scan', observed_scan, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/ci/ground_truth/odom', ground_truth, qos_profile_sensor_data)
        node.create_subscription(JointState, '/platform/motors/cmd', motor_command, qos_profile_sensor_data)
        node.create_subscription(String, '/test/collision_status',
                                 lambda m: observer.collision(sim_time(), m.data), 100)

        def spin(check_clock=True):
            nonlocal printed_events
            process_failure = os.environ.get('CI_PROCESS_FAILURE_FILE')
            if check_clock and process_failure and Path(process_failure).exists():
                raise RuntimeError(Path(process_failure).read_text().strip())
            rclpy.spin_once(node, timeout_sec=0.02)
            observer.tick(sim_time())
            for event in observer.events[printed_events:]:
                log('OBSERVER EVENT ' + json.dumps(event, sort_keys=True))
            printed_events = len(observer.events)
            for name, detail in observer.failures.items():
                if name not in printed_failures:
                    log(f'OBSERVER FAIL {name}: {detail}')
                    # GitHub renders this as a job annotation; escape control text.
                    escaped = str(detail).replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')
                    print(f'::error title=Observer {name}::{escaped}', flush=True)
                    printed_failures.add(name)
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
            observer.fault(sim_time(), enabled)
            log(f"FAULT {fault_id}: {'ENABLED' if enabled else 'DISABLED'}; sim={state['sim']:.3f}s")
            events.append({'id': fault_id, 'active': enabled, 'wall_elapsed': time.monotonic() - started, 'sim_time': state['sim']})

        def pose():
            transform = buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if state['sim'] is None or abs(state['sim'] - stamp) > 2:
                raise RuntimeError('Map pose transform is stale')
            return transform.transform

        log('Waiting for individual readiness checks')
        deadline = time.monotonic() + config['ready_timeout']
        last_readiness_log = 0
        while True:
            spin(False)
            process_failure = os.environ.get('CI_PROCESS_FAILURE_FILE')
            if process_failure and Path(process_failure).exists():
                raise RuntimeError(Path(process_failure).read_text().strip())
            try:
                pose()
                tf_ready = True
            except (TransformException, RuntimeError):
                tf_ready = False
            now = time.monotonic()
            checks = {
                'clock': state['sim'] is not None and now - state['advance'] < 1,
                'raw_scan': now - state['scan'] < 1,
                'map_tf': tf_ready,
                'nav2_action': client.server_is_ready(),
                'fault_service': service.service_is_ready(),
            }
            checks.update({name: name in observer.seen and
                           sim_time() - observer.seen[name] <= observer.limits.telemetry_timeout
                           for name in ('scan', 'command', 'motion', 'collision')})
            if checks != readiness or now - last_readiness_log >= 5:
                log('READINESS ' + ' '.join(f'{k}={"READY" if v else "WAIT"}' for k, v in checks.items()))
                last_readiness_log = now
            readiness = checks
            if all(checks.values()):
                break
            if now >= deadline:
                raise RuntimeError('Readiness timeout; missing/stale: ' +
                                   ', '.join(k for k, v in checks.items() if not v))
        # Action discovery precedes lifecycle activation; wait for all servers.
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            readiness[name] = False
            log(f'Waiting for {name} lifecycle activation')
            lifecycle = node.create_client(GetState, f'/{name}/get_state')
            while True:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Readiness timeout: {name} is not active')
                if lifecycle.service_is_ready():
                    response = wait(lifecycle.call_async(GetState.Request()), seconds=min(5, max(0.01, deadline - time.monotonic())))
                    if response.current_state.id == 3:
                        readiness[name] = True
                        log(f'READINESS {name}=ACTIVE')
                        break
                for _ in range(10):
                    spin()
        readiness['diff_drive_controller'] = False
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        wait_for_drive_controller(controllers, ListControllers.Request(), spin, log, deadline)
        readiness['diff_drive_controller'] = True
        parameters = node.create_client(GetParameters, '/collision_monitor/get_parameters')
        while not parameters.service_is_ready():
            spin()
            if time.monotonic() >= deadline:
                raise RuntimeError('Readiness timeout: Collision Monitor parameter service missing')
        actual = wait(parameters.call_async(GetParameters.Request(names=['source_timeout']))).values
        if len(actual) != 1 or not math.isclose(actual[0].double_value, observer.limits.scan_timeout, abs_tol=1e-6):
            raise RuntimeError('Collision Monitor source_timeout differs from observer scan_timeout; rebuild/source the updated rover workspace')
        log(f'STOP CONTRACT scan_timeout={observer.limits.scan_timeout}s command_margin={observer.limits.command_margin}s braking={observer.limits.braking_allowance}s')
        fault_state('ci_scan_dropout', False)
        log('Waiting for observer telemetry: injected scan, wheel commands, Unity ground truth and collisions')
        while True:
            spin()
            try:
                observer.arm(sim_time())
                break
            except ValueError as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Observer readiness timeout: {error}; rebuild the CI player')
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
            enabled_at = {}
            moving_since = None
            recovery_deadline = None
            original_recorded = False
            retry_sent = False
            record['attempts'] = []
            while True:
                spin()
                elapsed = time.monotonic() - schedule_start
                # Reset for each accepted goal; require fresh commanded and actual
                # movement for a full second, not residual velocity at acceptance.
                fresh_motion = (sim_time() - observer.seen.get('motion', -math.inf) < 0.2
                                and sim_time() - observer.seen.get('command', -math.inf) < 0.2)
                moving = (fresh_motion and not observer.is_stopped() and observer.command
                          and max(observer.command) > observer.limits.command_epsilon)
                if moving and not pending.done():
                    if moving_since is None:
                        moving_since = sim_time()
                else:
                    moving_since = None
                if time.monotonic() - goal_start > goal['timeout']:
                    raise RuntimeError(f"Goal deadline exceeded: {goal['name']}")
                for fault in goal.get('faults', []):
                    fid = fault['id']
                    if (elapsed >= fault['start'] and fid not in enabled and moving_since is not None
                            and sim_time() - moving_since >= 1.0):
                        fault_state(fid, True)
                        enabled.add(fid)
                        enabled_at[fid] = time.monotonic()
                    if fid in enabled and time.monotonic() - enabled_at[fid] >= fault['duration'] and fid not in disabled:
                        fault_state(fid, False)
                        disabled.add(fid)
                        recovery_deadline = time.monotonic() + 20.0
                if pending.done() and not original_recorded:
                    original = pending.result()
                    record['attempts'].append(dict(status=original.status,
                        error_code=getattr(original.result, 'error_code', None),
                        error_msg=getattr(original.result, 'error_msg', ''),
                        elapsed=time.monotonic() - goal_start))
                    original_recorded = True
                    log('NAVIGATION ATTEMPT ' + json.dumps(record['attempts'][-1]))
                    if not enabled and goal.get('faults'):
                        raise RuntimeError('Goal ended before sustained movement allowed injection')
                # Keep spinning after an abort until every activated outage has
                # run its full duration. Cleanup is reserved for real errors.
                if enabled - disabled:
                    continue
                if enabled:
                    healthy = observer.recovered
                    try:
                        pose()
                    except (TransformException, RuntimeError):
                        healthy = False
                    if not healthy:
                        if recovery_deadline is not None and time.monotonic() >= recovery_deadline:
                            raise RuntimeError('Recovery timeout: require healthy scans and fresh map TF')
                        continue
                    if pending.done() and pending.result().status == GoalStatus.STATUS_ABORTED and not retry_sent:
                        log('RECOVERY: healthy scans and fresh TF; retrying aborted goal once')
                        record['recovery_policy'] = 'retry_once_after_healthy_scans_and_tf'
                        message.pose.header.stamp = node.get_clock().now().to_msg()
                        handle = wait(client.send_goal_async(message, feedback_callback=feedback))
                        if not handle.accepted:
                            raise RuntimeError('Recovery retry rejected')
                        pending = handle.get_result_async()
                        retry_sent = True
                        original_recorded = False
                        continue
                if pending.done():
                    break
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
        # Drain post-motion collision heartbeats before declaring success.
        settle = time.monotonic() + 2.0
        while time.monotonic() < settle:
            spin()
        observer.navigation_result(sim_time(), True)
    except (Exception, KeyboardInterrupt) as error:
        failure = f'{type(error).__name__}: {error}'
        print(f'FAIL: {failure}', flush=True)
        escaped = failure.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')
        print(f'::error title=Navigation runner::{escaped}', flush=True)
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
            if observer is not None and observer.active:
                if failure:
                    observer.fail('runner', failure)
                observer_report = observer.finish(sim_time())
            node.destroy_node()
            rclpy.shutdown()
        if observer_report is None:
            observer_report = {'passed': False, 'failures': {'setup': failure or 'Observer never armed'}}
        write_report(observer_report, output / 'observer')
        log('OBSERVER ' + ('PASS' if observer_report['passed'] else 'FAIL') + ': ' + json.dumps(observer_report.get('failures', {})))
        if not observer_report['passed']:
            failure = failure or 'Resilience observer assertions failed (see observer/results.json)'
        if failure:
            if not results or results[-1]['passed']:
                results.append({'name': 'setup_or_cleanup', 'passed': False})
            results[-1]['error'] = failure
        report = {'passed': failure is None, 'elapsed': time.monotonic() - started, 'goals': results, 'fault_events': events,
                  'observer': observer_report, 'readiness': readiness}
        (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
        summary = navigation_summary(report)
        (output / 'summary.md').write_text(summary)
        if os.environ.get('GITHUB_STEP_SUMMARY'):
            with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as stream:
                stream.write(summary)
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
