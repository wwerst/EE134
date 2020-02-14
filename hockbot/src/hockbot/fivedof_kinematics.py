#!/usr/bin/env python
#
#   fivedof_kinematics.py

import rospy

import numpy as np
from scipy.spatial.transform import Rotation

from urdf_parser_py.urdf import URDF
import rospkg

URDF_PATH = rospkg.RosPack().get_path('hockbot') + '/urdf/robot_5dof.urdf'
robot = URDF.from_xml_file(URDF_PATH)
joints = robots.joints

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

def fkin(q0, q1, q2, q3, q4, q5):
    '''
    Returns (x, y, z, theta, phi, grip) as a function of actuator positions.
    q0-q4 take values in [0, 2Pi)
    q5 takes values 0 or 1 (electromagnet off or on)

    theta represents yaw of the gripper around the world z axis
    phi represents pitch of gripper WRT the horizon (world xy plane)
    '''
    
    # start before Joint 0
    x = vec(0,0,0)
    R = Rz(0)

    for joint in joints:
        # check for origin and axis
        if joint.origin:
            xyz = joint.origin.xyz
            rpy = joint.origin.rpy
            x = np.add(x, xyz)
            
        if joint.axis:
            xyz = joint.axis.xyz

    return [x[0], x[1], x[2], theta, phi, grip]