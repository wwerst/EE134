#!/usr/bin/env python
#
#   puck_dynamics.py

import numpy as np
from numpy.polynomial.polynomial import Polynomial
import shapely.geometry as sp_geom

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


class PuckDynamics(object):
    '''
    Object for keeping track of puck position given a set of known points
    '''
    def __init__(self, time_ref):
        '''
        Initialize the puck dynamics object with constants for the environment
        '''
        self.NUM_POINTS = 3 # Number of points to fit
        self.TABLE_LEFT = 0.0 # x Position of table left edge, meters
        self.TABLE_RIGHT = 1000.0 # x Position of the table right edge, meters
        self.TABLE_BOTTOM = 0.0 # y Position of table bottom edge, meters
        self.TABLE_TOP = 1000.0 # y Position of table top edge, meters
        self.PUCK_RADIUS = 1 # Puck radius, meters
        self.ELASTICITY = 0.5 # Elasticity coefficient, unitless

        # Storage for timestamps and x/y positions of puck
        self.t_arr = np.zeros(self.NUM_POINTS)
        self.x_arr = np.zeros(self.NUM_POINTS)
        self.y_arr = np.zeros(self.NUM_POINTS)

        # Initialize the polynomials
        self.x_coeffs = np.array([-1, 0])
        self.y_coeffs = np.array([-1, 0])

        # Present position and velocity of puck since last update
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)

        # Initialization decider variable
        self.num_valid_points = 0

        # Hold the time reference
        self.time_ref = time_ref

    def add_point(self, point_stamped):
        '''
        Adds an x, y pair to the list of most recent points with a timestamp
        '''
        # Extract the points from the message
        x = point_stamped.point.x
        y = point_stamped.point.y
        t = (point_stamped.header.stamp - self.time_ref).to_sec()

        # Keep track of number of valid points
        self.num_valid_points += 1

        # Shift all entries down 1
        self.t_arr = np.roll(self.t_arr, -1)
        self.x_arr = np.roll(self.x_arr, -1)
        self.y_arr = np.roll(self.y_arr, -1)

        # and add the most recent entry to the end
        np.put(self.t_arr, self.NUM_POINTS-1, t)
        np.put(self.x_arr, self.NUM_POINTS-1, x)
        np.put(self.y_arr, self.NUM_POINTS-1, y)

        # Recompute the regression
        if (self.num_valid_points >= self.NUM_POINTS):
            x_coeffs = Polynomial.fit(self.t_arr, self.x_arr, 2, full=True)
            y_coeffs = Polynomial.fit(self.t_arr, self.y_arr, 2, full=True)
            if x_coeffs[1][0] <= 0.01:
                self.x_coeffs = x_coeffs[0].convert().coef
            if y_coeffs[1][0] <= 0.01:
                self.y_coeffs = y_coeffs[0].convert().coef

        # Fill in present position and velocity
        self.position = np.array([x, y])
        if self.num_valid_points >= self.NUM_POINTS:
            # vx = np.polyval(np.array([2*self.x_coeffs[0], self.x_coeffs[1]]), self.t_arr[-1])
            # vy = np.polyval(np.array([2*self.y_coeffs[0], self.y_coeffs[1]]), self.t_arr[-1])
            time_constant = 0.3
            self.velocity[0] *= (1-time_constant)
            self.velocity[0] += time_constant * 0.5 * (
                    ((self.x_arr[self.NUM_POINTS-1] - self.x_arr[self.NUM_POINTS-2]) / 
                    (self.t_arr[self.NUM_POINTS-1] - self.t_arr[self.NUM_POINTS-2])) + 
                    ((self.x_arr[self.NUM_POINTS-2] - self.x_arr[self.NUM_POINTS-3]) / 
                    (self.t_arr[self.NUM_POINTS-2] - self.t_arr[self.NUM_POINTS-3])) 
                    )
            self.velocity[1] *= (1-time_constant)
            self.velocity[1] += time_constant * 0.5 * (
                    ((self.y_arr[self.NUM_POINTS-1] - self.y_arr[self.NUM_POINTS-2]) / 
                    (self.t_arr[self.NUM_POINTS-1] - self.t_arr[self.NUM_POINTS-2])) + 
                    ((self.y_arr[self.NUM_POINTS-2] - self.y_arr[self.NUM_POINTS-3]) / 
                    (self.t_arr[self.NUM_POINTS-2] - self.t_arr[self.NUM_POINTS-3]))
                    )
        else:
            self.velocity = np.zeros(2)
        self.x_coeffs = np.array([self.velocity[0], self.position[0] - self.velocity[0]*self.t_arr[-1]])
        self.y_coeffs = np.array([self.velocity[1], self.position[1] - self.velocity[1]*self.t_arr[-1]])

    def predict_position(self, t):
        '''
        Predicts the position of the puck as a function of the requested robot time
        '''

        # Account for no reverse direction due to slowdown. Assumes vertex is always in future
#        if t >= (-1.0 * self.x_coeffs[1])/(2.0 * self.x_coeffs[0]):
#            x = np.polyval(self.x_coeffs, (-1.0 * self.x_coeffs[1])/(2.0 * self.x_coeffs[0]))
#        else:
#            x = np.polyval(self.x_coeffs, t)
#
#        if t >= (-1.0 * self.y_coeffs[1])/(2.0 * self.y_coeffs[0]):
#            y = np.polyval(self.y_coeffs, (-1.0 * self.y_coeffs[1])/(2.0 * self.y_coeffs[0]))
#        else:
#            y = np.polyval(self.y_coeffs, t)
#
#        # Account for hitting walls, reflect with some elasticity coefficient
#        if x < self.TABLE_LEFT + self.PUCK_RADIUS:
#            x = (self.TABLE_LEFT + ((self.TABLE_LEFT - (x - self.PUCK_RADIUS)) * self.ELASTICITY) 
#            + self.PUCK_RADIUS)
#        elif x > self.TABLE_RIGHT - self.PUCK_RADIUS:
#            x = (self.TABLE_RIGHT - ((x + self.PUCK_RADIUS - self.TABLE_RIGHT) * self.ELASTICITY) 
#            - self.PUCK_RADIUS)
#
#        if y < self.TABLE_BOTTOM + self.PUCK_RADIUS:
#            y = (self.TABLE_BOTTOM + ((self.TABLE_BOTTOM - (y - self.PUCK_RADIUS)) * self.ELASTICITY) 
#            + self.PUCK_RADIUS)
#        elif y > self.TABLE_TOP - self.PUCK_RADIUS:
#            y = (self.TABLE_TOP - ((y + self.PUCK_RADIUS - self.TABLE_TOP) * self.ELASTICITY) 
#            - self.PUCK_RADIUS)

        x = np.polyval(self.x_coeffs, t)
        y = np.polyval(self.y_coeffs, t)

        retPoint = Point()
        if (self.num_valid_points >= self.NUM_POINTS):
            retPoint.x = x
            retPoint.y = y
        else:
            retPoint.x = self.position[0]
            retPoint.y = self.position[1]
        return retPoint


def intersect_line_segment_polygon(line_segment, polygon):
    """Find the nearest intersection of the given vector with given polygon."""
    sp_line_seg = sp_geom.LineString(line_segment)
    sp_polygon = sp_geom.LineString(polygon)
    start_point = sp_geom.Point(*line_segment[0])
    intersect_points = sp_polygon.intersection(sp_line_seg)
    min_point = None
    if isinstance(intersect_points, sp_geom.LineString):
        return None
    if isinstance(intersect_points, sp_geom.Point):
        return np.array(intersect_points)
    for p in intersect_points:
        if min_point is None or p.distance(start_point) < min_point.distance(start_point):
            min_point = p
    if min_point is None:
        return None
    return np.array(min_point)
