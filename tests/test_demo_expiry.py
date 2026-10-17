"""Expiry must finish the recorded trial without sending another action."""
import json
from geo_kin import DemoExpiredError
from vr_robosuite import cli
from vr_robosuite.control import RobotControl


def test_expiry_finishes_trial_cleanly(monkeypatch, tmp_path):
    def expired(self, frame):
        raise DemoExpiredError('Demo time limit reached')

    monkeypatch.setattr(RobotControl, 'action', expired)
    result_path = tmp_path / 'result.json'
    args = cli.parser().parse_args([
        '--device', 'replay', '--no-stream', '--steps', '1',
        '--width', '64', '--height', '64', '--output', str(tmp_path / 'trial'),
        '--result-file', str(result_path),
    ])
    result = cli.run(args)
    assert result['status'] == 'demo_expired'
    assert json.loads(result_path.read_text())['status'] == 'demo_expired'
