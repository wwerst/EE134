#!/usr/bin/env python
#
#   fourdof_kinematics.py

from collections import namedtuple

import numpy as np
from scipy.spatial.transform import Rotation

import rosgraph
import rospy
import rospkg
from urdf_parser_py.urdf import URDF


def vec(x, y, z):
    return np.array([x, y, z])


def Rx(q):
    return Rotation.from_rotvec(q * vec(1, 0, 0))


def Ry(q):
    return Rotation.from_rotvec(q * vec(0, 1, 0))


def Rz(q):
    return Rotation.from_rotvec(q * vec(0, 0, 1))


def get_urdf(file_path='/urdf/robot_5dof.urdf'):
    if rosgraph.is_master_online():
        return URDF.from_parameter_server()
    else:
        urdf_file = rospkg.RosPack().get_path('hockbot') + file_path
        return URDF.from_xml_file(urdf_file)


def get_joint(urdf, segment_name):
    for j in urdf.joints:
        if j.name == segment_name:
            return j
    return None


def get_link(urdf, link_name):
    for l in urdf.links:
        if l.name == link_name:
            return l
    return None


IntPose = namedtuple('IntPose', 'position orientation')


def fkin(joints, joint_prefix='Picker', base_frame='world', target_frame='tip'):
    joint_dict = {'{}/q{}'.format(joint_prefix, i): j for i, j in enumerate(joints)}
    robot_urdf = get_urdf()
    assert len(joints) == 5
    kinematic_chain = robot_urdf.get_chain(base_frame, target_frame)
    chain_position = np.array([0.0]*3)
    chain_orientation = Rz(0)
    for segment_name in kinematic_chain:
        link = get_link(robot_urdf, segment_name)
        if not link:
            joint = get_joint(robot_urdf, segment_name)
            if not joint:
                raise ValueError(
                    'segment {} missing'.format(
                        segment_name))
            origin = joint.origin
            if not joint.origin:
                pos_shift = np.array([0.0]*3)
                rot_matrix = Rz(0)
            else:
                pos_shift = np.array(joint.origin.xyz)
                rot_matrix = (
                    Rx(joint.origin.rpy[0])
                    * Ry(joint.origin.rpy[1])
                    * Rz(joint.origin.rpy[2]))
            if joint.type != 'fixed':
                assert joint.type == 'continuous' or joint.type == 'revolute'
                # This is a rotary joint
                assert joint.axis == [0, 0, 1], 'Only support joints rotating around z-axis'
                rot_matrix *= Rz(joint_dict[joint.name])
            chain_position += chain_orientation.apply(pos_shift)
            chain_orientation *= rot_matrix
            print('*** Joint {} ***'.format(segment_name))
            print(chain_position)
            print(chain_orientation.as_quat())

    # print(robot_urdf)
    return None


def ikin(x, y, z, theta, phi):
    pass


if __name__ == '__main__':
    print(fkin([-1.58, 1.54, 0.90, -1.00, 0.27]))