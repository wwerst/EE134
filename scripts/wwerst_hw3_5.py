#!/usr/bin/env python2

from copy import deepcopy
from threading import Lock

import numpy as np
import rospy
from ee134.msg import GimbalPosition
from sensor_msgs.msg import JointState


def get_joint_pos_ordered(ordered_names):
    jstate = get_joint_state()
    positions = np.array([0.0]*len(ordered_names))
    for i, i_name in enumerate(ordered_names):
        for j, j_name in enumerate(jstate.name):
            if i_name == j_name:
                positions[i] = jstate.position[j]
    return np.array(positions)


def get_joint_state():
    return rospy.wait_for_message(
        '/hebiros/robot/feedback/joint_state',
        JointState)


def limit_array(arr, limit):
    arr = np.minimum(arr, np.array([limit]*len(arr)))
    arr = np.maximum(arr, np.array([-limit]*len(arr)))
    return arr


class GimbalManager(object):
    """Subscribe to GimbalPosition commands and relay commands to motors."""

    def __init__(self, yaw_axis_name, pitch_axis_name):
        self._lock = Lock()
        self._yaw_axis_name = yaw_axis_name
        self._pitch_axis_name = pitch_axis_name
        self._cmd_pos = get_joint_pos_ordered(['Chewbacca/Yaw', 'Chewbacca/Pitch'])
        self._cur_pos = deepcopy(self._cmd_pos)
        self._cur_vel = np.array([0.0]*len(self._cmd_pos))
        self._cur_acc = np.array([0.0]*len(self._cmd_pos))
        self._pub = rospy.Publisher(
            '/hebiros/robot/command/joint_state',
            JointState,
            queue_size=1)
        self._goal_sub = rospy.Subscriber(
            '/gimbal_goal',
            GimbalPosition,
            self._set_angles_cb)
        rate = 0.01
        self._last_time = rospy.Time.now().to_sec()
        self._timer = rospy.Timer(
            rospy.Duration(rate),
            self._send_joints)

    def set_angles(self, yaw, pitch):
        with self._lock:
            self._cmd_pos = np.array([yaw, pitch])

    def _set_angles_cb(self, msg):
        self.set_angles(msg.longitude, msg.latitude)

    def _send_joints(self, _):
        dt = rospy.Time.now().to_sec() - self._last_time
        self._last_time += dt
        command_msg = JointState()
        command_msg.name = [self._yaw_axis_name, self._pitch_axis_name]
        time_constant = 0.1      # Convergence time constant
        lam = 1.0/time_constant   # Convergence rate
        damping = 1.5
        max_acc = 1500.0
        max_velocity = 3.0              # Velocity magnitude limit
        with self._lock:
            self._cur_acc = - 1.4 * damping * lam * self._cur_vel - lam * lam * (self._cur_pos - self._cmd_pos)
            self._cur_acc = limit_array(self._cur_acc, max_acc)
            self._cur_vel += dt * self._cur_acc
            # Apply velocity limits
            self._cur_vel = limit_array(self._cur_vel, max_velocity)

            self._cur_pos += dt * self._cur_vel
            command_msg.position = self._cur_pos
            command_msg.velocity = self._cur_vel
            command_msg.effort = np.array([0.0]*len(self._cur_pos))
            command_msg.header.stamp = rospy.Time.now()
        self._pub.publish(command_msg)


def main():
    rospy.init_node('hw3_5')
    manager = GimbalManager('Chewbacca/Yaw', 'Chewbacca/Pitch')
    rospy.spin()


if __name__ == '__main__':
    main()
