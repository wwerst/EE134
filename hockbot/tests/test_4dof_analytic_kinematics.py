
import unittest

import numpy as np

from hockbot.fourdof_kinematics import (
    FourDoFKinematics,
    SURFACE_COLLECT,
    SURFACE_PLAY,
)


class Test4DoFKinematics(unittest.TestCase):

    def test_fkin(self):
        kin = FourDoFKinematics()
        theta0 = np.pi/2
        theta1 = (-1.0/6)*np.pi
        theta2 = (-2.0/3)*np.pi
        theta3 = (-3.0/2)*np.pi - theta1 - theta2
        res = kin.fkin(theta0, theta1, theta2, theta3)
        assert np.allclose(res, [-2.4492935982947072e-17, -0.40000000000000013, 9.265260047530536e-17, -4.71238898038469])

    def test_ikin(self):
        kin = FourDoFKinematics()
        b = kin.ikin(0, -0.4, 0, surface=SURFACE_COLLECT)
        print(b)
        assert np.allclose(b, [1.5707963267948966, 0.5235987755982987, -2.0943951023931957, 2.0943951023931957])
