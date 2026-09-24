"""ROS-independent assertion engine. All timestamps are simulation seconds."""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass
class Limits:
    scan_timeout: float = 0.5
    command_margin: float = 0.1
    braking_allowance: float = 0.5
    telemetry_timeout: float = 0.5
    max_stop_distance: float = 0.30
    linear_epsilon: float = 0.02
    angular_epsilon: float = 0.02
    command_epsilon: float = 0.001
    healthy_scans: int = 3
    min_dropout: float = 2.0


class Observer:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        for key, value in asdict(self.limits).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.active = False
        self.failures = {}
        self.events = []
        self.seen = {}
        self.last_scan = None
        self.scan_stamp = None
        self.position = None
        self.speed = None
        self.command = None
        self.collision_state = None
        self.fault_started = None
        self.fault_ended = None
        self.fault_active = False
        self.stop_origin = None
        self.stop_distance = 0.0
        self.max_measured_stop_distance = 0.0
        self.command_stop = None
        self.motion_stop = None
        self.recovery_count = 0
        self.recovered = False
        self.navigation_success = False
        self.last_time = None
        self.fault_scan_count = 0

    def navigation_result(self, t, success):
        if not self.active:
            return
        if not success:
            self.fail('navigation_result', 'Navigation runner reported failure')
        elif not self.recovered:
            self.fail('navigation_result', 'Navigation succeeded before scan recovery was established')
        else:
            self.navigation_success = True
        self.event(t, 'navigation_result', success=success)

    def fail(self, name, detail):
        if self.active:
            self.failures.setdefault(name, detail)

    def event(self, t, kind, **values):
        if self.active:
            self.events.append(dict(time=t, kind=kind, **values))

    def scan(self, t, stamp):
        if not math.isfinite(stamp) or stamp > t + 0.05 or t - stamp > self.limits.scan_timeout:
            self.fail('scan_timestamp', 'Scan timestamp is stale, future or invalid')
            return
        if self.scan_stamp is not None and stamp <= self.scan_stamp:
            self.fail('scan_timestamp', 'Scan timestamps did not advance')
            return
        previous = self.last_scan
        self.last_scan = t
        self.scan_stamp = stamp
        self.seen['scan'] = t
        if self.active and self.fault_active and t > self.fault_started + self.limits.command_margin:
            self.fault_scan_count += 1
            self.fail('dropout_confirmed', 'Scans arrived during the requested dropout')
        if self.active and self.fault_ended is not None:
            self.recovery_count = self.recovery_count + 1 if previous is not None and t - previous <= self.limits.scan_timeout else 1
            if self.recovery_count >= self.limits.healthy_scans and not self.recovered:
                self.recovered = True
                self.event(t, 'scans_recovered', count=self.recovery_count)

    def velocity(self, t, linear, angular):
        if not all(math.isfinite(v) for v in (linear, angular)):
            self.fail('command_valid', 'Non-finite velocity command')
            return
        self.seen['command'] = t
        self.command = (abs(linear), abs(angular))
        if self.fault_started is not None and not self.recovered:
            zero = max(self.command) <= self.limits.command_epsilon
            if zero and self.command_stop is None:
                self.command_stop = t
                self.event(t, 'command_stopped')
                if t > self.command_deadline:
                    self.fail('command_stop', 'First stop command arrived after deadline')
            if not zero and (self.command_stop is not None or t >= self.command_deadline):
                self.fail('command_stop', 'Nonzero command after stop/deadline and before recovery')

    def motion(self, t, x, y, linear, angular):
        if not all(math.isfinite(v) for v in (x, y, linear, angular)):
            self.fail('motion_valid', 'Non-finite ground-truth measurement')
            return
        if self.active and self.fault_started is not None and not self.recovered and self.position is not None:
            self.stop_distance += math.hypot(x - self.position[0], y - self.position[1])
            self.max_measured_stop_distance = max(self.max_measured_stop_distance, self.stop_distance)
            if self.stop_distance > self.limits.max_stop_distance:
                self.fail('stopping_distance', f'{self.stop_distance:.3f} m exceeds {self.limits.max_stop_distance:.3f} m')
        self.position = (x, y)
        self.speed = (abs(linear), abs(angular))
        self.seen['motion'] = t
        if self.fault_started is not None and not self.recovered:
            stopped = self.is_stopped()
            if stopped and self.motion_stop is None:
                self.motion_stop = t
                self.event(t, 'motion_stopped', distance=self.stop_distance)
                if t > self.motion_deadline:
                    self.fail('physical_stop', 'Physical stop observed after deadline')
            if not stopped and (self.motion_stop is not None or t >= self.motion_deadline):
                self.fail('physical_stop', 'Motion after stop/deadline and before recovery')

    def collision(self, t, payload):
        # A cumulative counter catches brief contacts even when event packets are lost.
        try:
            state = json.loads(payload) if isinstance(payload, str) else payload
            if not isinstance(state['session'], str) or not state['session']:
                raise ValueError('missing session')
            if type(state['count']) is not int or state['count'] < 0:
                raise ValueError('invalid counter')
            if type(state['active']) is not int or state['active'] < 0:
                raise ValueError('invalid active-contact count')
            if type(state['configured']) is not bool or not state['configured']:
                raise ValueError('collision publisher has no obstacle layers configured')
        except (ValueError, TypeError, KeyError) as exc:
            self.fail('collision_telemetry', str(exc))
            return
        previous = self.collision_state
        self.collision_state = dict(state)
        self.seen['collision'] = t
        if not self.active:
            return
        if previous and (state['session'] != previous['session'] or state['count'] < previous['count']):
            self.fail('collision_telemetry', 'Collision publisher restarted or counter decreased')
        if state['active'] or (previous and state['count'] > previous['count']):
            self.fail('no_collision', 'Unity reported obstacle contact')
            if previous is None or state['count'] != previous['count'] or state['active'] != previous['active']:
                self.event(t, 'collision', **state)

    def arm(self, t):
        if self.active:
            raise ValueError('Already armed; restart observer for a new scenario')
        for name in ('scan', 'motion', 'command', 'collision'):
            if name not in self.seen or t - self.seen[name] > self.limits.telemetry_timeout:
                raise ValueError(f'Missing or stale {name} telemetry')
        if self.collision_state['active']:
            raise ValueError('Rover already touching an obstacle')
        self.active = True
        self.last_time = t
        self.event(t, 'armed')

    def fault(self, t, enabled):
        if not self.active:
            return
        if enabled:
            if self.fault_started is not None:
                if not self.fault_active:
                    self.fail('fault_sequence', 'Only one dropout per observer run is supported')
                return
            if self.last_scan is None or t - self.last_scan > self.limits.scan_timeout:
                self.fail('precondition', 'No fresh scan at injection')
            if self.speed is None or self.is_stopped():
                self.fail('precondition', 'Rover was not moving at injection')
            self.fault_started = t
            self.fault_active = True
            # Deadline is deliberately conservative: last received scan before fault event.
            self.stop_origin = self.last_scan if self.last_scan is not None else t
            self.event(t, 'fault_enabled')
        elif self.fault_active:
            self.fault_active = False
            self.fault_ended = t
            self.event(t, 'fault_disabled')
            if t - self.fault_started < self.limits.min_dropout:
                self.fail('dropout_duration', 'Outage was shorter than min_dropout')

    @property
    def command_deadline(self):
        return self.stop_origin + self.limits.scan_timeout + self.limits.command_margin

    @property
    def motion_deadline(self):
        return self.command_deadline + self.limits.braking_allowance

    def is_stopped(self):
        return self.speed is not None and self.speed[0] <= self.limits.linear_epsilon and self.speed[1] <= self.limits.angular_epsilon

    def tick(self, t):
        if not self.active:
            return
        if self.last_time is not None and t < self.last_time:
            self.fail('clock', 'Simulation clock moved backwards')
        self.last_time = t
        for name in ('motion', 'command', 'collision'):
            if t - self.seen.get(name, -math.inf) > self.limits.telemetry_timeout:
                self.fail('telemetry_' + name, f'{name} heartbeat missing or stale')
        if self.recovered and t - self.last_scan > self.limits.scan_timeout:
            self.fail('recovery_stable', 'Scans became stale again after recovery')
        if self.fault_started is not None:
            if t >= self.command_deadline and self.command_stop is None:
                self.fail('command_stop', 'No observed stop command before deadline')
            if t >= self.motion_deadline and self.motion_stop is None:
                self.fail('physical_stop', 'No observed physical stop before deadline')

    def finish(self, t):
        if not self.active:
            raise ValueError('Observer must be armed before finishing')
        self.tick(t)
        required = {
            'no_collision': 'no_collision' not in self.failures and 'collision_telemetry' not in self.failures and 'telemetry_collision' not in self.failures,
            'telemetry_complete': not any(k.startswith('telemetry_') for k in self.failures),
            'fault_exercised': self.fault_started is not None and self.fault_ended is not None,
            'command_stop_observed': self.command_stop is not None,
            'physical_stop_observed': self.motion_stop is not None,
            'scans_recovered': self.recovered,
            'navigation_success': self.navigation_success,
        }
        for name, passed in required.items():
            if not passed:
                self.fail(name, 'Required evidence was not observed')
        self.event(t, 'finished')
        return {
            'passed': not self.failures, 'failures': dict(self.failures),
            'checks': required, 'limits': asdict(self.limits),
            'metrics': {'stop_distance_m': self.max_measured_stop_distance,
                        'command_stop_time': self.command_stop, 'motion_stop_time': self.motion_stop},
            'events': list(self.events),
        }


def write_report(report, directory):
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True)
    (path / 'results.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    suite = ET.Element('testsuite', name='rover_resilience', tests='1', failures=str(int(not report['passed'])))
    case = ET.SubElement(suite, 'testcase', name='dropout_stop_recovery_and_no_collision')
    if not report['passed']:
        ET.SubElement(case, 'failure', message='Observer assertions failed').text = json.dumps(report['failures'], indent=2)
    ET.ElementTree(suite).write(path / 'junit.xml', encoding='utf-8', xml_declaration=True)
