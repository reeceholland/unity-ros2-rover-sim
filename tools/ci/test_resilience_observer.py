import json
import tempfile
import unittest
from pathlib import Path
from resilience_observer import Observer, write_report


def collision(count=0, active=0, session='run-1'):
    return dict(session=session, count=count, active=active, configured=True, last_object='wall')


class ObserverTests(unittest.TestCase):
    def ready(self):
        o = Observer()
        o.scan(1.0, 1.0)
        o.motion(1.0, 0, 0, .2, 0)
        o.velocity(1.0, .2, 0)
        o.collision(1.0, collision())
        o.arm(1.0)
        return o

    def heartbeat(self, o, t, count=0):
        o.motion(t, .05, 0, 0, 0)
        o.velocity(t, 0, 0)
        o.collision(t, collision(count))
        o.tick(t)

    def complete(self, o):
        o.fault(1.0, True)
        for i in range(1, 21):
            self.heartbeat(o, 1.0 + i * .1)
        o.fault(3.0, False)
        for t in (3.1, 3.2, 3.3):
            o.scan(t, t)
            self.heartbeat(o, t)
        o.navigation_result(3.3, True)
        return o.finish(3.3)

    def test_healthy_run_passes(self):
        self.assertTrue(self.complete(self.ready())['passed'])

    def test_collision_stays_failed_after_contact_ends(self):
        o = self.ready()
        o.collision(1.01, collision(1, 1))
        o.collision(1.02, collision(1, 0))
        self.assertIn('no_collision', o.failures)

    def test_missed_collision_event_detected_from_counter(self):
        o = self.ready()
        o.collision(1.1, collision(2, 0))
        self.assertIn('no_collision', o.failures)

    def test_publisher_restart_fails(self):
        o = self.ready()
        o.collision(1.1, collision(session='new'))
        self.assertIn('collision_telemetry', o.failures)

    def test_missing_collision_heartbeat_fails(self):
        o = self.ready()
        o.tick(1.6)
        self.assertIn('telemetry_collision', o.failures)

    def test_cannot_arm_without_collision_telemetry(self):
        with self.assertRaises(ValueError):
            Observer().arm(1)

    def test_cannot_arm_in_contact(self):
        o = Observer()
        o.scan(1, 1); o.motion(1, 0, 0, 0, 0); o.velocity(1, 0, 0)
        o.collision(1, collision(1, 1))
        with self.assertRaises(ValueError):
            o.arm(1)

    def test_scans_during_dropout_fail(self):
        o = self.ready(); o.fault(1, True); o.scan(1.3, 1.3)
        self.assertIn('dropout_confirmed', o.failures)

    def test_late_stop_callback_cannot_hide_missed_deadline(self):
        o = self.ready(); o.fault(1, True)
        o.velocity(2, 0, 0); o.motion(2.3, .1, 0, 0, 0)
        self.assertIn('command_stop', o.failures)
        self.assertIn('physical_stop', o.failures)

    def test_path_length_not_net_displacement(self):
        o = self.ready(); o.fault(1, True)
        o.motion(1.1, .2, 0, .2, 0); o.motion(1.2, 0, 0, .2, 0)
        self.assertIn('stopping_distance', o.failures)

    def test_restart_before_healthy_scans_fails(self):
        o = self.ready(); o.fault(1, True)
        for i in range(1, 21):
            self.heartbeat(o, 1 + i * .1)
        o.fault(3, False)
        o.velocity(3.1, .2, 0); o.motion(3.1, .06, 0, .2, 0)
        self.assertIn('command_stop', o.failures)
        self.assertIn('physical_stop', o.failures)

    def test_transient_stop_during_braking_is_not_a_failure(self):
        o = self.ready(); o.fault(1, True)
        self.heartbeat(o, 1.1)
        o.velocity(1.2, .2, 0); o.motion(1.2, .06, 0, .2, 0)
        self.assertFalse(o.failures)
        self.assertIsNone(o.command_stop)
        self.assertIsNone(o.motion_stop)
        for i in range(3, 21):
            self.heartbeat(o, 1 + i * .1)
        self.assertFalse(o.failures)
        self.assertIsNotNone(o.command_stop)
        self.assertIsNotNone(o.motion_stop)

    def test_single_zero_cannot_establish_settled_stop(self):
        o = self.ready(); o.fault(1, True)
        self.heartbeat(o, 1.5)
        o.tick(3)
        self.assertIsNone(o.command_stop)
        self.assertIsNone(o.motion_stop)
        self.assertIn('command_stop', o.failures)

    def test_restart_after_deadline_fails(self):
        o = self.ready(); o.fault(1, True)
        for i in range(1, 17):
            self.heartbeat(o, 1 + i * .1)
        o.velocity(2.7, .2, 0); o.motion(2.7, .06, 0, .2, 0)
        self.assertIn('command_stop', o.failures)
        self.assertIn('physical_stop', o.failures)

    def test_stale_replayed_scan_not_healthy(self):
        o = self.ready(); o.scan(1.1, 1)
        self.assertIn('scan_timestamp', o.failures)

    def test_stationary_fault_is_not_valid_test(self):
        o = self.ready(); o.motion(1, 0, 0, 0, 0); o.fault(1, True)
        self.assertIn('precondition', o.failures)

    def test_unexercised_test_fails(self):
        o = self.ready()
        self.assertFalse(o.finish(1)['passed'])

    def test_clock_reset_fails(self):
        o = self.ready(); o.tick(.5)
        self.assertIn('clock', o.failures)

    def test_navigation_success_before_recovery_fails(self):
        o = self.ready(); o.navigation_result(1.1, True)
        self.assertIn('navigation_result', o.failures)

    def test_unconfigured_collision_publisher_rejected(self):
        o = self.ready()
        state = collision(); state['configured'] = False
        o.collision(1.1, state)
        self.assertIn('collision_telemetry', o.failures)

    def test_report_files(self):
        report = self.complete(self.ready())
        with tempfile.TemporaryDirectory() as path:
            write_report(report, path)
            self.assertTrue(json.loads((Path(path) / 'results.json').read_text())['passed'])
            self.assertIn('failures="0"', (Path(path) / 'junit.xml').read_text())


if __name__ == '__main__':
    unittest.main()
