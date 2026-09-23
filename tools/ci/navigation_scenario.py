"""ROS-independent validation and pose checks for headless scenarios."""
import json
import math


def positive(value, name, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if value < 0 or (not allow_zero and value == 0):
        raise ValueError(f"{name} must be {'nonnegative' if allow_zero else 'positive'}")


def load_scenario(path):
    with open(path, encoding='utf-8') as stream:
        data = json.load(stream)
    if data.get('frame_id') != 'map':
        raise ValueError('Goals must use the ROS map frame')
    for key in ('ready_timeout', 'clock_timeout'):
        positive(data[key], key)
    if not data.get('goals'):
        raise ValueError('At least one goal is required')
    names = set()
    for goal in data['goals']:
        if not isinstance(goal.get('name'), str) or not goal['name'] or goal['name'] in names:
            raise ValueError('Goal names must be nonempty and unique')
        names.add(goal['name'])
        for key in ('x', 'y', 'yaw_deg'):
            value = goal[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
        for key in ('timeout', 'position_tolerance', 'yaw_tolerance_deg'):
            positive(goal[key], key)
        ids = set()
        for fault in goal.get('faults', []):
            if fault.get('id') != 'ci_scan_dropout' or fault['id'] in ids:
                raise ValueError('Use each supported fault (ci_scan_dropout) at most once per goal')
            ids.add(fault['id'])
            positive(fault['start'], 'fault start', allow_zero=True)
            positive(fault['duration'], 'fault duration')
            if fault['start'] + fault['duration'] >= goal['timeout']:
                raise ValueError('Fault must finish before the goal deadline')
    return data


def pose_errors(goal, x, y, yaw):
    distance = math.hypot(x - goal['x'], y - goal['y'])
    delta = yaw - math.radians(goal['yaw_deg'])
    angle = abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))
    return distance, angle
