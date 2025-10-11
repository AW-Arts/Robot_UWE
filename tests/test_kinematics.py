from __future__ import annotations

import math

import numpy as np

from lynxmotion_control.al5a_kinematics import AL5AKinematics, joints_to_pulses


def test_forward_inverse_roundtrip():
    kin = AL5AKinematics()
    joints = [math.radians(angle) for angle in [0, 45, -30, 20]]
    pose = kin.forward(joints)
    result = kin.inverse(pose[:3, 3], wrist_pitch=joints[1] + joints[2] + joints[3])
    np.testing.assert_allclose(result, joints, atol=1e-6)


def test_joints_to_pulses_monotonic():
    pulses = joints_to_pulses([0.0, 0.5, -1.0, 0.1, 0.0])
    assert pulses[0] > 500
    assert pulses[1] > 500
    assert pulses[2] < 2000


def test_inverse_limits_reachable():
    kin = AL5AKinematics()
    pose = kin.inverse([0.15, 0.0, 0.12], wrist_pitch=math.radians(30))
    assert len(pose) == 4
    assert all(math.isfinite(angle) for angle in pose)
