#!/usr/bin/env python3
"""Check full scan dropout and recovery against a running simulator and injector."""
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ros2_fault_injection.srv import SetFaultState


def main():
    rclpy.init()
    node = rclpy.create_node('test_ci_scan_dropout')
    counts = [0, 0]
    stamps = [set(), set()]

    def receive(msg, index):
        if not msg.ranges:
            return
        counts[index] += 1
        stamps[index].add((msg.header.stamp.sec, msg.header.stamp.nanosec))

    subscriptions = [node.create_subscription(
        LaserScan, topic, lambda m, i=i: receive(m, i), qos_profile_sensor_data)
        for i, topic in enumerate(('/scan_raw', '/ci/scan'))]
    client = node.create_client(SetFaultState, '/fault_injection/set_fault_state')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def set_fault(active):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Fault-state service unavailable')
        request = SetFaultState.Request(fault_id='ci_scan_dropout', active=active)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('Fault-state service timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)

    def window(seconds):
        counts[:] = [0, 0]
        stamps[0].clear()
        stamps[1].clear()
        spin(seconds)
        return counts.copy(), len(stamps[0] & stamps[1])

    code = 1
    try:
        deadline = time.monotonic() + 60
        while min(counts) < 5 and time.monotonic() < deadline:
            spin(0.1)
        if min(counts) < 5:
            raise RuntimeError('Readiness timeout: raw and forwarded scans required')
        set_fault(False)
        baseline, matched = window(2)
        if min(baseline) < 5 or matched < 5:
            raise RuntimeError(f'Baseline forwarding failed: {baseline}, matched={matched}')
        set_fault(True)
        spin(0.5)  # Drain messages queued before activation was acknowledged.
        dropped, _ = window(2)
        if dropped[0] < 5 or dropped[1] != 0:
            raise RuntimeError(f'Dropout failed: raw/output={dropped}')
        set_fault(False)
        recovered, matched = window(3)
        if min(recovered) < 5 or matched < 5:
            raise RuntimeError(f'Recovery failed: {recovered}, matched={matched}')
        print(f'PASS: baseline={baseline}, dropout={dropped}, recovery={recovered}')
        code = 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'FAIL: {error}')
    finally:
        try:
            set_fault(False)
        except Exception as error:
            print(f'Cleanup could not disable fault: {error}')
            code = 1
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
