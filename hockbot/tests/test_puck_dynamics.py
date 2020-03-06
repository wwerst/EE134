import unittest

import numpy as np
import shapely.geometry as sp_geom

from hockbot.puck_dynamics import PuckDynamics, intersect_line_segment_polygon

import rospy
from geometry_msgs.msg import PointStamped

# TODO other things now


class TestPuckDynamics(unittest.TestCase):

    def test_puck_intersection(self):
        puck_pos = np.array([1.5, 1.0])
        puck_vel = np.array([-1.0, -1.0])
        triangle_defense_poly = np.array([[0.5, 0], [0.75, 0.5], [1.0, 0.0]])
        intersect_pos = intersect_line_segment_polygon(
            np.array([puck_pos, puck_pos + puck_vel]),
            triangle_defense_poly)
        assert np.allclose(intersect_pos, np.array([5./6, 1./3]))
    # def test_normal(self):
    #     pd = PuckDynamics()
    #     testpoints = []
    #     t = 0
    #     pos = 5
    #     for i in range(20):
    #         point_stamped = PointStamped()
    #         point_stamped.header.stamp.sec = t
    #         point_stamped.point.x = pos
    #         point_stamped.point.y = pos

    #         t += 0.1
    #         pos -= t**2


    # def test_slowdown(self):
    #     pd = PuckDynamics()
    #     kin = FourDoFKinematics()
    #     theta0 = np.pi/2
    #     theta1 = (-1.0/6)*np.pi
    #     theta2 = (-2.0/3)*np.pi
    #     theta3 = (-3.0/2)*np.pi - theta1 - theta2
    #     res = kin.fkin(theta0, theta1, theta2, theta3)
    #     assert np.allclose(res, [-1.11136697e-17, -1.81500000e-01, 4.16687730e-01, 5.23598776e-01])

    # def test_reflection(self):
    #     pd = PuckDynamics()
    #     kin = FourDoFKinematics()
    #     b = kin.ikin(0, -0.4, 0, surface=SURFACE_COLLECT)
    #     print(b)
    #     assert np.allclose(b, [1.57079633, 0.93042535, -1.38978214, 2.39218149])
