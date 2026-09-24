#!/usr/bin/env python3
"""Move the CI rover forward for 30 seconds and verify motion and sensor data."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan


class MotionTest:
    def __init__(self, args):
        self.args = args

        self.node = rclpy.create_node('test_ci_rover_motion')

        self.sim = None
        self.previous_sim = None
        self.clock_reset = False

        self.odom = None
        self.feedback = None

        self.seen = {}
        self.scans = 0

        self.cmd = self.node.create_publisher(
            TwistStamped,
            args.command_topic,
            10
        )

        self.subs = [
            self.node.create_subscription(
                Clock,
                '/clock',
                self.clock,
                qos_profile_sensor_data
            ),

            self.node.create_subscription(
                Odometry,
                args.odom_topic,
                self.pose,
                qos_profile_sensor_data
            ),

            self.node.create_subscription(
                LaserScan,
                args.scan_topic,
                self.scan,
                qos_profile_sensor_data
            ),

            self.node.create_subscription(
                JointState,
                args.feedback_topic,
                self.wheels,
                qos_profile_sensor_data
            ),
        ]

    def clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.sim is not None and value < self.sim:
            self.clock_reset = True

        if self.sim is None or value > self.sim:
            self.seen['clock'] = time.monotonic()

        self.sim = value

    def pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        if not all(
            math.isfinite(v)
            for v in (p.x, p.y, q.x, q.y, q.z, q.w)
        ):
            return

        self.odom = msg
        self.seen['odom'] = time.monotonic()

    def scan(self, msg):
        if msg.ranges and any(math.isfinite(v) for v in msg.ranges):
            self.scans += 1
            self.seen['scan'] = time.monotonic()

    def wheels(self, msg):
        required = (
            'front_left_joint',
            'front_right_joint',
            'rear_left_joint',
            'rear_right_joint',
        )

        values = dict(zip(msg.name, msg.velocity))

        if all(
            k in values and math.isfinite(values[k])
            for k in required
        ):
            self.feedback = [values[k] for k in required]
            self.seen['feedback'] = time.monotonic()

    def fresh(self):
        now = time.monotonic()

        return all(
            now - self.seen.get(k, -math.inf) < 2.0
            for k in ('clock', 'odom', 'scan', 'feedback')
        )

    def spin(self):
        rclpy.spin_once(
            self.node,
            timeout_sec=0.02
        )

        if self.clock_reset:
            raise RuntimeError(
                'Simulation clock reset during the test'
            )

    def command(self, speed):
        msg = TwistStamped()

        msg.header.frame_id = 'base_link'

        if self.sim is not None:
            msg.header.stamp.sec = int(self.sim)
            msg.header.stamp.nanosec = int(
                (self.sim - int(self.sim)) * 1e9
            )

        msg.twist.linear.x = speed

        self.cmd.publish(msg)

    def stop(self):
        # Use wall time so stopping is still attempted
        # even if /clock pauses.
        deadline = time.monotonic() + 1.0

        while time.monotonic() < deadline and rclpy.ok():
            self.command(0.0)

            rclpy.spin_once(
                self.node,
                timeout_sec=0.05
            )

    def run(self):

        # ---------------------------------------------------------
        # Wait for simulation and sensors to become ready
        # ---------------------------------------------------------

        deadline = time.monotonic() + self.args.ready_timeout
        first_sim = None

        while time.monotonic() < deadline:
            self.spin()

            if first_sim is None:
                first_sim = self.sim

            if (
                self.fresh()
                and self.cmd.get_subscription_count() > 0
                and first_sim is not None
                and self.sim > first_sim
            ):
                break

        else:
            raise RuntimeError(
                'Readiness timeout: need advancing /clock, scans, '
                'odometry, wheel feedback and a command subscriber'
            )

        # ---------------------------------------------------------
        # Store initial position
        # ---------------------------------------------------------

        initial = self.odom.pose.pose

        q = initial.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        start = self.sim
        scans = self.scans

        print(
            f'Starting motion test: '
            f'{self.args.speed:.3f} m/s for '
            f'{self.args.travel_time:.1f} seconds'
        )

        # ---------------------------------------------------------
        # Drive forward
        # ---------------------------------------------------------

        deadline = (
            time.monotonic()
            + self.args.motion_timeout
        )

        next_command = 0.0

        while self.sim - start < self.args.travel_time:

            self.spin()

            if time.monotonic() > deadline:
                raise RuntimeError(
                    'Motion timeout'
                )

            if not self.fresh():
                raise RuntimeError(
                    'Sensor data became stale during motion'
                )

            # Re-publish velocity command at approximately 20 Hz.
            if time.monotonic() >= next_command:

                self.command(self.args.speed)

                next_command = (
                    time.monotonic() + 0.05
                )

        motion_duration = self.sim - start

        # ---------------------------------------------------------
        # Stop rover
        # ---------------------------------------------------------

        self.stop()

        # ---------------------------------------------------------
        # Calculate displacement
        # ---------------------------------------------------------

        final = self.odom.pose.pose.position

        dx = final.x - initial.position.x
        dy = final.y - initial.position.y

        forward = (
            dx * math.cos(yaw)
            + dy * math.sin(yaw)
        )

        straight_line_distance = math.sqrt(
            dx * dx + dy * dy
        )

        expected_distance = (
            self.args.speed * motion_duration
        )

        effective_velocity = (
            forward / motion_duration
            if motion_duration > 0.0
            else 0.0
        )

        # ---------------------------------------------------------
        # Diagnostics
        # ---------------------------------------------------------

        print()
        print('Motion results')
        print('--------------')

        print(
            f'Commanded speed:       '
            f'{self.args.speed:.3f} m/s'
        )

        print(
            f'Motion duration:       '
            f'{motion_duration:.3f} s'
        )

        print(
            f'Nominal distance:      '
            f'{expected_distance:.3f} m'
        )

        print(
            f'Forward displacement:  '
            f'{forward:.3f} m'
        )

        print(
            f'Straight-line distance: '
            f'{straight_line_distance:.3f} m'
        )

        print(
            f'Effective velocity:    '
            f'{effective_velocity:.3f} m/s'
        )

        print(
            f'Valid scans received:  '
            f'{self.scans - scans}'
        )

        # ---------------------------------------------------------
        # Verify rover actually moved
        # ---------------------------------------------------------

        if forward < self.args.minimum_distance:
            raise RuntimeError(
                f'Expected at least '
                f'{self.args.minimum_distance:.2f} m '
                f'forward displacement, '
                f'got {forward:.3f} m'
            )

        # ---------------------------------------------------------
        # Verify scans continued while driving
        # ---------------------------------------------------------

        if self.scans - scans < 5:
            raise RuntimeError(
                'Too few valid scans during motion'
            )

        # ---------------------------------------------------------
        # Verify wheels stop
        # ---------------------------------------------------------

        deadline = time.monotonic() + 5.0
        settled_since = None

        while time.monotonic() < deadline:

            self.command(0.0)

            self.spin()

            if (
                self.fresh()
                and self.feedback is not None
                and max(abs(v) for v in self.feedback) < 0.1
            ):

                if settled_since is None:
                    settled_since = time.monotonic()

                if (
                    time.monotonic() - settled_since
                    >= 0.5
                ):
                    print()
                    print(
                        f'PASS: moved {forward:.3f} m '
                        f'in {motion_duration:.3f} s; '
                        f'scans continued; wheels stopped'
                    )

                    return

            else:
                settled_since = None

        raise RuntimeError(
            'Wheels did not settle below 0.1 rad/s'
        )


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        '--command-topic',
        default='/diff_drive_controller/cmd_vel'
    )

    parser.add_argument(
        '--odom-topic',
        default='/odom'
    )

    parser.add_argument(
        '--scan-topic',
        default='/scan_raw'
    )

    parser.add_argument(
        '--feedback-topic',
        default='/platform/motors/feedback'
    )

    parser.add_argument(
        '--ready-timeout',
        type=float,
        default=30.0
    )

    # Must be longer than the 30-second travel period.
    parser.add_argument(
        '--motion-timeout',
        type=float,
        default=45.0
    )

    # Rover velocity command.
    parser.add_argument(
        '--speed',
        type=float,
        default=0.15
    )

    # Drive for 30 seconds of simulation time.
    parser.add_argument(
        '--travel-time',
        type=float,
        default=30.0
    )

    # Only verify that meaningful forward movement occurred.
    parser.add_argument(
        '--minimum-distance',
        type=float,
        default=0.10
    )

    args = parser.parse_args()

    rclpy.init()

    test = MotionTest(args)

    result = 1

    try:
        test.run()
        result = 0

    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'FAIL: {error}')

    finally:

        try:
            test.stop()

        finally:
            test.node.destroy_node()
            rclpy.shutdown()

    return result


if __name__ == '__main__':
    raise SystemExit(main())