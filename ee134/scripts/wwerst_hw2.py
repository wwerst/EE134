#!/usr/bin/env python2
#
#   pysendcommands.py
#
#   Continually (at 100Hz!) send commands to the robot
#   (in hebiros_node).

import sys
import rospy
import math

from hebiros.srv import AddGroupFromNamesSrv, EntryListSrv, SizeSrv
from sensor_msgs.msg import JointState


MOVE_AMPLITUDE = math.pi/4
SINE_FREQUENCY = 0.25
JOINT_INFO = None


def get_joint_info():
    global JOINT_INFO
    if JOINT_INFO is None:
        JOINT_INFO = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)().entry_list.entries[0]
    return JOINT_INFO


def initialize_robot():
    # You can choose to wait until the hebiros_node is running (or
    # else fail the below).
    rospy.loginfo("Waiting for the hebiros_node...")
    rospy.wait_for_service('/hebiros/add_group_from_names')

    # To communicate with the Hebi node, we create a proxy for their
    # service.  Note the service arguments (to be communicated both to
    # and from the service) are set by hebiros::AddGroupFromNamesSrv.
    proxy = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)

    # if get_robot_size() == 1:
    #     rospy.loginfo('Robot already defined, not redefining')
    #     return
    # Call the service.
    rospy.loginfo("Calling /hebiros/add_group_from_names...")
    try:
        resp = proxy(
            group_name='robot',
            names={get_joint_info().name},
            families={get_joint_info().family})
        rospy.loginfo("'robot' created")
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

    size = get_robot_size()
    if size != 1:
        raise RuntimeError('Error initializing robot')


def get_robot_size():
    # Let's check the resulting robot's number of actuators...
    rospy.wait_for_service('/hebiros/robot/size')
    size_proxy = rospy.ServiceProxy('/hebiros/robot/size', SizeSrv);
    try:
        resp = size_proxy()
        rospy.loginfo("'robot' has size %d" % resp.size);
        return resp.size
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)


def compute_trajectory_point(starting_pos, amplitude, dt, hz=1.0):
    print('dt = {}'.format(dt))
    command_msg = JointState()
    command_msg.name.append('{family}/{name}'.format(
        family=get_joint_info().family,
        name=get_joint_info().name))
    command_msg.position.append(starting_pos + MOVE_AMPLITUDE*(math.cos(dt*2*math.pi*hz) - 1))
    print(command_msg.position[0])
    command_msg.velocity.append(MOVE_AMPLITUDE*-(2*math.pi*hz)*math.sin(dt*2*math.pi*hz))
    acceleration = -MOVE_AMPLITUDE*((2*math.pi*hz)**2) * math.cos(dt*2*math.pi*hz)
    command_msg.effort.append(0.0*acceleration)
    return command_msg


def move_sine_forever(hz):
    # Create a publisher to send commands to the robot.  Also
    # initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=1)

    start_pos = rospy.wait_for_message(
        '/hebiros/robot/feedback/joint_state',
        JointState).position[0]
    print('Starting position {}'.format(start_pos))
    # Create a servo loop at 100Hz.
    servo = rospy.Rate(100)
    dt = servo.sleep_dur.to_sec()

    # Run the servo loop until shutdown.
    rospy.loginfo("Running the servo loop with dt %f" % dt)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()
        ramped_t = t**2 / (0.2+t)

        command_msg = compute_trajectory_point(
            start_pos, MOVE_AMPLITUDE, ramped_t, hz)
        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()


def main():
    # Initialize the basic ROS node.
    rospy.init_node('wwerst_hw2')

    initialize_robot()
    move_sine_forever(SINE_FREQUENCY)


if __name__ == "__main__":
    main()
