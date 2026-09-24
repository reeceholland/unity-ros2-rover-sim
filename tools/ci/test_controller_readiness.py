from types import SimpleNamespace
import unittest
from controller_readiness import wait_for_drive_controller


class ControllerReadinessTests(unittest.TestCase):
    def run_case(self, states, available=True, responds=True):
        clock = [0.0]
        calls = []
        logs = []
        cancelled = []

        def query(request):
            state = states[min(len(calls), len(states)-1)]
            calls.append(state)
            items = [] if state is None else [SimpleNamespace(name='diff_drive_controller', state=state)]
            return SimpleNamespace(done=lambda: responds,
                                   result=lambda: SimpleNamespace(controller=items),
                                   cancel=lambda: cancelled.append(True))

        client = SimpleNamespace(service_is_ready=lambda: available, call_async=query)
        def spin():
            clock[0] += .05
        def wait():
            wait_for_drive_controller(client, object(), spin, logs.append, 1.0, lambda: clock[0])
        return wait, calls, logs, cancelled

    def test_waits_through_loading_and_inactive_states(self):
        wait, calls, logs, _ = self.run_case([None, 'inactive', 'active'])
        wait()
        self.assertEqual(calls, [None, 'inactive', 'active'])
        self.assertEqual(logs[-1], 'READINESS diff_drive_controller=ACTIVE')

    def test_inactive_controller_cannot_pass(self):
        wait, _, _, _ = self.run_case(['inactive'])
        with self.assertRaisesRegex(RuntimeError, 'not active'):
            wait()

    def test_missing_service_is_bounded(self):
        wait, calls, _, _ = self.run_case(['active'], available=False)
        with self.assertRaisesRegex(RuntimeError, 'service unavailable'):
            wait()
        self.assertFalse(calls)

    def test_unanswered_service_is_bounded(self):
        wait, calls, _, cancelled = self.run_case(['active'], responds=False)
        with self.assertRaisesRegex(RuntimeError, 'response'):
            wait()
        self.assertEqual(len(calls), 1)
        self.assertTrue(cancelled)


if __name__ == '__main__':
    unittest.main()
