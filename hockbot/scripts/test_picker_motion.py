#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

import actionlib
from hockbot.srv import (
    MovePickerCart,
)

def main():
    rospy.init_node('test_picker_motion')
    picker_move_service_name = '/move_picker_cart'
    rospy.wait_for_service(picker_move_service_name)
    move_picker = rospy.ServiceProxy(picker_move_service_name, MovePickerCart)
    move_picker(x=0.4, y=0.2, z=0.3, tool_vertical=True, cartesian_interp=True, move_time=4.0)
    while not rospy.is_shutdown():
        move_picker(
            x=0.1,
            y=0.1,
            z=0.2,
            tool_vertical=True,
            cartesian_interp=True,
            move_time=4.0)
        move_picker(
            x=-0.2,
            y=0.4,
            z=0.3,
            tool_vertical=True,
            cartesian_interp=True,
            move_time=3.0)
        move_picker(
            x=-0.2,
            y=0.4,
            z=0.03,
            tool_vertical=True,
            cartesian_interp=True,
            move_time=1.5)
        move_picker(
            x=-0.2,
            y=0.4,
            z=0.3,
            tool_vertical=True,
            cartesian_interp=True,
            move_time=2.5)
        move_picker(
            x=0.4,
            y=0.2,
            z=0.3,
            tool_vertical=True,
            cartesian_interp=True,
            move_time=4.0)


if __name__ == '__main__':
    main()
