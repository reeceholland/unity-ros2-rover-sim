"""Bounded controller-manager readiness, independent of ROS imports."""
import time


def wait_for_drive_controller(client, request, spin, log, deadline, now=time.monotonic):
    last_state = None
    pending = None
    next_query = 0
    while now() < deadline:
        spin()
        state = 'service unavailable'
        if client.service_is_ready():
            state = 'waiting for controller-manager response'
            if pending is None and now() >= next_query:
                pending = client.call_async(request)
            if pending is not None and pending.done():
                response = pending.result()
                controllers = {item.name: item.state for item in response.controller}
                state = controllers.get('diff_drive_controller', 'not loaded')
                pending = None
                next_query = now() + 0.25
                if state == 'active':
                    log('READINESS diff_drive_controller=ACTIVE')
                    return
            elif pending is None:
                state = last_state
        if state != last_state:
            log(f'READINESS diff_drive_controller=WAIT ({state})')
            last_state = state
    if pending is not None:
        pending.cancel()
    raise RuntimeError(f'Readiness timeout: diff_drive_controller is not active ({last_state})')
