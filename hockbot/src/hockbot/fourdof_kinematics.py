#!/usr/bin/env python
#
#   fourdof_kinematics.py

import rosgraph
import rospy
import numpy as np
from urdf_parser_py.urdf import URDF
import rospkg


# Functional constants
SURFACE_PLAY = 0
SURFACE_COLLECT = 1


def get_urdf(file_path='/urdf/robot_4dof.urdf'):
    if rosgraph.is_master_online():
        urdf = URDF.from_parameter_server()
    else:
        urdf_file = rospkg.RosPack().get_path('hockbot') + file_path
        urdf = URDF.from_xml_file(urdf_file)
    return urdf

class FourDoFConstants(object):

    def __init__(self):
        joints = get_urdf().joints
        # Robot constants
        self.X_OFFSET = joints[0].origin.xyz[0]
        self.Y_OFFSET = joints[0].origin.xyz[1]
        self.Z_OFFSET = joints[2].origin.xyz[2]

        self.TIP_LENGTH = np.sqrt(
            joints[-1].origin.xyz[0]**2 +
            joints[-1].origin.xyz[1]**2 +
            joints[-1].origin.xyz[2]**2)
            # Only one of these should be nonzero anyway,
            # but compute full length for completeness

        self.L1 = joints[4].origin.xyz[0]
        self.L2 = joints[6].origin.xyz[0]


class FourDoFKinematics(object):

    def __init__(self):
        self.CONST = FourDoFConstants()

    def fkin(self, q0, q1, q2, q3):
        '''
        Returns (x, y, z, theta, grip) as a function of actuator positions.
        q0-q3 take values in [0, 2Pi)

        theta represents pitch of gripper WRT vertical
        '''

        # HACK negate some joint
        q1 = -q1

        q3 = -q3


        # Radial distance from base
        R = self.CONST.L1*np.sin(q1) + self.CONST.L2*np.sin(q1 + q2) + self.CONST.TIP_LENGTH*np.sin(q1 + q2 + q3)

        # and height above robot z
        z = self.CONST.L1*np.cos(q1) + self.CONST.L2*np.cos(q1 + q2) + self.CONST.TIP_LENGTH*np.cos(q1 + q2 + q3)

        # x and y displacements from robot origin
        x = R*np.cos(q0)
        y = R*np.sin(q0)

        # Adjust for world coordinates
        x = x + self.CONST.X_OFFSET
        y = y + self.CONST.Y_OFFSET
        z = z + self.CONST.Z_OFFSET

        # Calculate theta
        theta = q1 + q2 + q3

        return np.array([x, y, z, theta])

    def ikin(self, x, y, z, surface=SURFACE_PLAY):
        '''
        Returns (q0, q1, q2, q3) as a function of requested cartesian position.

        If surface = SURFACE_PLAY then (x, y, z) refers to a position in the playing area relative to
        the bottom left corner. Tip will be pointed directly in the -z direction. +z is the height off
        the xy plane.

        If surface = SURFACE_COLLECT then x is irrelevant and (y, z) represent the position of the
        gripper relative to the above origin. -z goes below the xy plane (to get the puck) and -y
        brings the gripper away from the side of the table. The gripper tip will be pointed directly 
        at the xz plane.

        Returns None if position out of bounds or out of reach
        '''

        # Work from the robot's reference frame
        x = x - self.CONST.X_OFFSET
        y = y - self.CONST.Y_OFFSET
        z = z - self.CONST.Z_OFFSET

        # Check out of bounds/reach
        if (surface == SURFACE_PLAY):
            if (x**2 + y**2 + (z + self.CONST.TIP_LENGTH)**2 > (self.CONST.L1+self.CONST.L2)**2): # Cannot reach longer than arms
                print('Robot cannot reach')
                return None
            elif (y < 0): # Cannot reach behind itself in play mode
                print('Cant go behind robot')
                return None
            elif (z < -1 * self.CONST.Z_OFFSET): # Don't crash into the table
                print('Crash into table')
                return None

        if (surface == SURFACE_COLLECT):
            if (z**2 + self.CONST.TIP_LENGTH**2 > (self.CONST.L1+self.CONST.L2)**2 or np.absolute(y) > self.CONST.L1 + self.CONST.L2): # Cannot reach longer than arms
                print('Robot cannot reach')
                return None
            elif (not np.allclose(x, 0)): # Need to reach in the x=0 plane
                print('Robot cannot reach out of x=0 plane')
                return None
            elif (z > 0): # Don't want to use these kinematics in the z>0 regime
                print('Cannot go above xy plane')
                return None
            elif (y > -1 * self.CONST.Y_OFFSET): # Don't crash into the side of the table
                print('Crash into table')
                return None

        # Calculate the kinematics
        if (surface == SURFACE_PLAY):
            # Easy to calculate the base angle, as it's just the rotation necessary to hit the point on 
            # the xy plane
            q0 = np.arctan2(y, x)

            # Calculate the 2 joints above the base joint

            R = np.sqrt(x**2 + y**2) # Distance in the cartesian plane from base to tip touchdown
            r = np.sqrt(R**2 + (z + self.CONST.TIP_LENGTH)**2) # Distance from q1 to q3 joints

            q2 = np.arccos((r**2 - self.CONST.L1**2 - self.CONST.L2**2)/(2*self.CONST.L1*self.CONST.L2)) # Law of cosines solves q2

            # Difference of angle to pointer and interior angle
            q1 = np.arctan2(R,(z + self.CONST.TIP_LENGTH)) - np.arctan2((self.CONST.L2*np.sin(q2)),(self.CONST.L1 + self.CONST.L2*np.cos(q2)))

            q3 = np.pi - q2 - q1 # Net 180 degree rotation

        elif (surface == SURFACE_COLLECT):
            q0 = np.pi/2.0 # Fix the base joint to point directly forward

            # Calculate the 2 joints behind the base joint

            r = np.sqrt((y - self.CONST.TIP_LENGTH)**2 + z**2) # Distance from q1 to q3 joints

            q2 = -1 * np.arccos((r**2 - self.CONST.L1**2 - self.CONST.L2**2)/(2*self.CONST.L1*self.CONST.L2)) # Law of cosines solves q2

            # Difference of angle to pointer and interior angle
            q1 = (np.arctan2(np.absolute(y - self.CONST.TIP_LENGTH), np.absolute(z)) + np.arctan2((self.CONST.L2*np.sin(np.absolute(q2))),(self.CONST.L1 + self.CONST.L2*np.cos(q2)))) - np.pi

            q3 = (-3.0/2)*np.pi - q1 - q2 # Net 270 degree backwards rotation

        return np.array([q0, -q1, q2, -q3])

