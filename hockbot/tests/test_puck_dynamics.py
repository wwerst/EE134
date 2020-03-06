import unittest

import numpy as np

from hockbot.puck_dynamics import PuckDynamics

import rospy
from geometry_msgs.msg import PointStamped

# TODO other things now

class TestPuckDynamics(unittest.TestCase):

    def test_normal(self):
        pd = PuckDynamics()
        testpoints = []
        t = 0
        pos = 5
        for i in range(20):
            point_stamped = PointStamped()
            point_stamped.header.stamp.sec = t
            point_stamped.point.x = pos
            point_stamped.point.y = pos

            t += 0.1
            pos -= t**2


    def test_slowdown(self):
        pd = PuckDynamics()
        kin = FourDoFKinematics()
        theta0 = np.pi/2
        theta1 = (-1.0/6)*np.pi
        theta2 = (-2.0/3)*np.pi
        theta3 = (-3.0/2)*np.pi - theta1 - theta2
        res = kin.fkin(theta0, theta1, theta2, theta3)
        assert np.allclose(res, [-1.11136697e-17, -1.81500000e-01, 4.16687730e-01, 5.23598776e-01])

    def test_reflection(self):
        pd = PuckDynamics()
        kin = FourDoFKinematics()
        b = kin.ikin(0, -0.4, 0, surface=SURFACE_COLLECT)
        print(b)
        assert np.allclose(b, [1.57079633, 0.93042535, -1.38978214, 2.39218149])