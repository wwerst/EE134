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
            if xyz[0]:
                R = R * Rx(joint_thetas[i])
            elif xyz[1]:
                R = R * Ry(joints_thetas[i])
            elif xyz[2]:
                R = R * Rz(joint_thetas[i])

            i = i + 1

    # Extract the tip position from the R matrix
    angles = R.as_euler('xyz')
    phi = angles[2] # Phi is just the z axis rotation
    theta = angles[1] # Theta is just the y axis rotation

    return (x[0], x[1], x[2], theta, phi, grip)