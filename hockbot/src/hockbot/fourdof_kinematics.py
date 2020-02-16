#!/usr/bin/env python
#
#   fourdof_kinematics.py

import rospy

import numpy as np
from scipy.spatial.transform import Rotation

from urdf_parser_py.urdf import URDF
import rospkg

URDF_PATH = rospkg.RosPack().get_path('hockbot') + '/urdf/robot_4dof.urdf'
robot = URDF.from_xml_file(URDF_PATH)
joints = robots.joints

# Constants ----------------------------------------------------------------------------------------

# Robot constants
X_OFFSET = joints[0].origin.xyz[0]
Y_OFFSET = joints[0].origin.xyz[1]
Z_OFFSET = joints[2].origin.xyz[2]

TIP_LENGTH = np.sqrt(   joints[-1].origin.xyz[0]**2 + 
                        joints[-1].origin.xyz[1]**2 + 
                        joints[-1].origin.xyz[2]**2) # Only one of these should be nonzero anyway, 
                                                     # but compute full length for completeness

L1 = joints[4].origin.xyz[0]
L2 = joints[6].origin.xyz[0]

# Functional constants
SURFACE_PLAY = 0
SURFACE_COLLECT = 1

#
#  Matrix Manipulation Utilities
#
def vec(x, y, z):
    return np.array([x, y, z])
def Rx(q):
    return Rotation.from_rotvec(q * vec(1, 0, 0))
def Ry(q):
    return Rotation.from_rotvec(q * vec(0, 1, 0))
def Rz(q):
    return Rotation.from_rotvec(q * vec(0, 0, 1))

def fkin(q0, q1, q2, q3, q4):
    '''
    Returns (x, y, z, theta, phi, grip) as a function of actuator positions.
    q0-q3 take values in [0, 2Pi)
    q4 takes values 0 or 1 (electromagnet off or on)

    phi represents yaw of the gripper around the world z axis
    theta represents pitch of gripper WRT the horizon (world xy plane)
    '''
    
    joint_thetas = [q0, q1, q2, q3, q4]
    i = 0

    # start before Joint 0
    x = vec(0,0,0)
    R = Rz(0)

    for joint in joints:
        # If joint has an origin field, it's just a translation and maybe rotation
        if joint.origin:
            xyz = joint.origin.xyz
            rpy = joint.origin.rpy
            x = x + R.apply(vec(xyz[0], xyz[1], xyz[2]))
            R = R * Rx(rpy[0]) * Ry(rpy[1]) * Rz(rpy[2])
            
        # if joint has an axis field, it's just a rotation about a motor
        if joint.axis:
            xyz = joint.axis.xyz
            R = R * 
                Rx(joint_thetas[i] * xyz[0]) * 
                Ry(joint_thetas[i] * xyz[1]) * 
                Rz(joint_thetas[i] * xyz[2])
            i = i + 1

    # Extract the tip position from the R matrix
    angles = R.as_euler('xyz')
    phi = angles[2] # Phi is just the z axis rotation
    theta = angles[1] # Theta is just the y axis rotation

    return (x[0], x[1], x[2], theta, phi, grip)

def ikin(x, y, z, surface):
    '''
    Returns (q0, q1, q2, q3) as a function of requested cartesian position.
    If surface = SURFACE_PLAY then (x, y) refers to a position in the playing area relative to
    the bottom left corner.
    If surface = SURFACE_COLLECT then x is irrelevant and y represents the (positive) normal 
    distance between the surface of the playing area and the center of the puck in the goal 
    collection area.

    Returns None if position out of bounds or out of reach
    '''

    # Work from the robot's reference frame
    x = x - X_OFFSET
    y = y - Y_OFFSET
    z = z - Z_OFFSET

    # Check out of bounds/reach
    if (surface == SURFACE_PLAY):
        if (x**2 + y**2 + (z + TIP_LENGTH)**2 > (L1+L2)**2): # Cannot reach longer than arms
            return None
        elif (y <= 0): # Cannot reach behind itself in play mode
            return None
        elif (z < TIP_LENGTH - Z_OFFSET): # Don't crash into the table
            return None

    if (surface == SURFACE_COLLECT):
            return None

    # Calculate the kinematics
    if (surface == SURFACE_PLAY):
        # Easy to calculate the base angle, as it's just the rotation necessary to hit the point on 
        # the xy plane
        q0 = np.arctan2(y, x)

        # Calculate the 3 joints above the base joint

        R = np.sqrt(x**2 + y**2) # Distance in the cartesian plane from base to tip touchdown
        r = np.sqrt(R**2 + z**2) # Distance from q1 to q3 joints

        q2 = np.arccos((r**2 - L1**2 - L2**2)/(2*L1*L2)) # Law of cosines solves q2 and q1

        # Difference of angle to pointer and interior angle
        q1 = np.arctan2(R/z) - np.arctan2((L2*np.sin(q2))/(L1 + L2*np.cos(q2)))

        q3 = np.pi - q2 - q1 # Net 180 degree rotation

    elif (surface == SURFACE_REACH):
        q0 = np.pi/2.0 # Fix the base joint to point directly forward

    return (q0, q1, q2, q3)
