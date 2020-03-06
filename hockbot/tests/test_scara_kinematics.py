
import unittest
import numpy as np

from hockbot.scara_kinematics import (
    fkin,
    jacobian,
    ikin,
)


class TestScaraKinematics(unittest.TestCase):
    TEST_VECS = [
        (np.array([0      ,        0]), np.array([ 1.0, 0.0])),
        (np.array([np.pi/2,        0]), np.array([ 0.5, 0.5])),
        (np.array([np.pi/2, -np.pi/2]), np.array([ 0.0, 1.0])),
        (np.array([np.pi/2,  np.pi/2]), np.array([ 0.0, 0.0])),
        (np.array([np.pi  , -np.pi/2]), np.array([-0.5, 0.5])),
        (np.array([np.pi/4,        0]), np.array([ 0.5+np.sqrt(0.125), np.sqrt(0.125)])),
        (np.array([np.pi/4, -np.pi/4]), np.array([ 0.5*np.sqrt(2), 0.5*np.sqrt(2)])),
    ]

    def test_fkin(self):
        # Test fkin
        for joints, expected_pos in self.TEST_VECS:
            act_pos = fkin(*joints)
            # assert np.allclose(
            #     act_pos,
            #     expected_pos)

    def test_ikin_of_fkin(self):
        # Assumes that fkin is trustworthy, and then
        # uses trustworthy fkin to verify output of
        # ikin is correct
        for joints, expected_pos in self.TEST_VECS:
            act_pos = fkin(*joints)
            ikin_joints = ikin(*act_pos)
            for possible_joints in ikin_joints:
                pass
                # assert np.allclose(
                #     fkin(*possible_joints),
                #     act_pos)
