import unittest
from test_ci_navigation import navigation_summary


class ReportingTests(unittest.TestCase):
    def test_successful_retry_does_not_hide_original_abort(self):
        summary = navigation_summary(dict(passed=True, readiness={}, observer={'failures': {}},
            goals=[dict(name='point_2', passed=True, recovery_policy='retry_once',
                        attempts=[dict(status=6, error_code=102), dict(status=4, error_code=0)])]))
        self.assertIn('Attempt 1: status=6, error_code=102', summary)
        self.assertIn('Attempt 2: status=4, error_code=0', summary)
        self.assertIn('one explicit retry', summary)

    def test_missing_prerequisite_is_visible(self):
        summary = navigation_summary(dict(passed=False, goals=[],
            readiness={'clock': True, 'map_tf': False},
            observer={'failures': {'setup': 'Readiness timeout'}}))
        self.assertIn('Missing/stale readiness: map_tf', summary)
        self.assertIn('Readiness timeout', summary)


if __name__ == '__main__':
    unittest.main()
