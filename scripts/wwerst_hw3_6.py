#!/usr/bin/env python2

from copy import deepcopy
from threading import Lock

import numpy as np
import rospy
from ee134.msg import GimbalPosition
from sensor_msgs.msg import JointState
from geometry_msgs_msg import PointStamped
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
        #self._face_sub = rospy.Subscriber(
        #    '/detector/faces',
        #    FaceArrayStamped,
        #    self._get_face_cb)
        self._striker_sub = rospy.Subscriber(
            '/detector/strikers',
            PointStamped,
            self._get_striker_cb)

    def _get_pos_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'Yaw' in name:
                self._last_long = msg.position[i]
            if 'Pitch' in name:
                self._last_lat = msg.position[i]

    def _get_face_cb(self, msg):
        RADIANS_PER_PIXEL = 1.0/640
        first_eye = None
        for f in msg.faces:
            for e in f.eyes:
                first_eye = e
                break
        if not first_eye:
            return
        latitude = self._last_lat - (first_eye.y-240.0) * RADIANS_PER_PIXEL
        longitude = self._last_long - (first_eye.x-320.0) * RADIANS_PER_PIXEL
        self._angle_pub.publish(GimbalPosition(latitude, longitude))

    def _get_striker_cb(self, msg):
        RADIANS_PER_PIXEL = 1.0/640
        latitude = self._last_lat - (msg.point.y-240.0) * RADIANS_PER_PIXEL
        longitude = self._last_long - (msg.point.x-320.0) * RADIANS_PER_PIXEL
        self._angle_pub.publish(GimbalPosition(latitude, longitude))


def main():
    rospy.init_node('hw3_6')
    manager = CameraManager()
    rospy.spin()


if __name__ == '__main__':
    main()
