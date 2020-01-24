#!/usr/bin/env python2

from copy import deepcopy
from threading import Lock

import numpy as np
import rospy
from ee134.msg import GimbalPosition
from sensor_msgs.msg import JointState
from opencv_apps.msg import FaceArrayStamped


def limit_array(arr, limit):
    arr = np.minimum(arr, np.array([limit]*len(arr)))
    arr = np.maximum(arr, np.array([-limit]*len(arr)))
    return arr


class CameraManager(object):
    """Subscribe to GimbalPosition commands and relay commands to motors."""

    def __init__(self):
        self._lock = Lock()
        self._last_lat = 0
        self._last_long = 0
        self._angle_pub = rospy.Publisher(
            '/gimbal_goal',
            GimbalPosition,
            queue_size=1)
        self._pos_sub = rospy.Subscriber(
            '/hebiros/robot/feedback/joint_state',
            JointState,
            self._get_pos_cb)
        self._face_sub = rospy.Subscriber(
            '/detector/faces',
            FaceArrayStamped,
            self._get_face_cb)

    def _get_pos_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'Yaw' in name:
                self._last_lat = msg.position[i]
            if 'Pitch' in name:
                self._last_long = msg.position[i]

    def _get_face_cb(self, msg):
        RADIANS_PER_PIXEL = 1.5/640
        if not msg.faces:
            return
        face = msg.faces[0].face
        latitude = self._last_lat - (face.x-320.0) * RADIANS_PER_PIXEL
        longitude = self._last_long - (face.y-240.0) * RADIANS_PER_PIXEL
        self._angle_pub.publish(GimbalPosition(latitude, longitude))

    def _send_joints(self, _):
        command_msg = JointState()
        command_msg.name = [self._yaw_axis_name, self._pitch_axis_name]
        time_constant = 0.2      # Convergence time constant
        lam = 1.0/time_constant   # Convergence rate
        damping = 2.0
        max_acc = 2.0
        max_velocity = 3.0              # Velocity magnitude limit
        with self._lock:
            self._cur_acc = - 1.4 * damping * lam * self._cur_vel - lam * lam * (self._cur_pos - self._cmd_pos)
            self._cur_acc = limit_array(self._cur_acc, max_acc)
            self._cur_vel += self._dt * self._cur_acc
            # Apply velocity limits
            self._cur_vel = limit_array(self._cur_vel, max_velocity)

            self._cur_pos += self._dt * self._cur_vel
            command_msg.position = self._cur_pos
            command_msg.velocity = self._cur_vel
            command_msg.effort = np.array([0.0]*len(self._cur_pos))
            command_msg.header.stamp = rospy.Time.now()
        self._pub.publish(command_msg)


def main():
    rospy.init_node('hw3_6')
    manager = CameraManager()
    rospy.spin()


if __name__ == '__main__':
    main()
