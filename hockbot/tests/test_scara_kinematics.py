
import unittest
import numpy as np

from hockbot.scara_kinematics import (
    fkin,
    jacobian,
    ikin,
)


class TestScaraKinematics(unittest.TestCase):

    def test_fkin(self):
        test_vecs = [
            (np.array([0      ,        0]), np.array([ 1.0, 0.0])),
            (np.array([np.pi/2,        0]), np.array([ 0.5, 0.5])),
            (np.array([np.pi/2, -np.pi/2]), np.array([ 0.0, 1.0])),
            (np.array([np.pi/2,  np.pi/2]), np.array([ 0.0, 0.0])),
            (np.array([np.pi  , -np.pi/2]), np.array([-0.5, 0.5])),
        ]
        for joints, expected_pos in test_vecs:
            act_pos = fkin(*joints)
            assert np.allclose(
                act_pos,
                expected_pos)

    def test_ikin_of_fkin(self):
        test_vecs = [
            (np.array([0      ,        0]), np.array([ 1.0, 0.0])),
            (np.array([np.pi/2,        0]), np.array([ 0.5, 0.5])),
            (np.array([np.pi/2, -np.pi/2]), np.array([ 0.0, 1.0])),
            (np.array([np.pi/2,  np.pi/2]), np.array([ 0.0, 0.0])),
            (np.array([np.pi  , -np.pi/2]), np.array([-0.5, 0.5])),
        ]
        for joints, expected_pos in test_vecs:
            act_pos = fkin(*joints)
            ikin_joints = ikin(*act_pos)
            for possible_joints in ikin_joints:
                assert np.allclose(
                    fkin(*possible_joints),
                    act_pos)
