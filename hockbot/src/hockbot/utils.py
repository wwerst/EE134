import numpy as np

import rospy
from sensor_msgs.msg import JointState


def remap_joint_state(joint_state, desired_name_order):
    new_state = JointState()
    new_state.name = desired_name_order
    new_state.position = [0]*5
    new_state.velocity = [0]*5
    new_state.effort = [0]*5
    for i, name in enumerate(new_state.name):
        ordered_index = new_state.name.index(name)
        new_state.position[ordered_index] = joint_state.position[i]
        new_state.velocity[ordered_index] = joint_state.velocity[i]
        new_state.effort[ordered_index] = joint_state.effort[i]
    return new_state


def quadratic_interp_joint_state(start_state, end_joints, move_time, dt=0.01):
    assert len(start_state.name) == end_joints
    start_joints = np.array(start_state.position)
    def interp_pos(frac):
        new_state = JointState()
        new_state.position = (
            start_joints
            (np.array(end_joints) - start_joints)*frac)
    positions = []
    for frac in np.linspace(0.0, 1.0, move_time//dt):
        smooth_frac = 2*frac**2 if frac < 0.5 else (-2*(frac-1)**2 + 1)
        jstate = interp_pos(smooth_frac)
        positions.append(jstate)
    return positions


def blocking_trianglewave(pub, cmdmsg, joint, amplitude, period, gravity=None):
    """Triangle Wave
    Move a single joint according to a triangle wave of given
    amplitude and period.  Use a gravity model if available.
    """
    # Report
    rospy.loginfo('Running triangle wave on jnt %d, amp %f, period %f ...' %
                  (joint, amplitude, period))

    # Make sure we have a positive period.
    period = max(period, 0.01)

    # Remember the center and pre-compute the speed.
    center = cmdmsg.position[joint]
    speed  = 4.0 * amplitude / period

    # Use a 100Hz servo to send the commands.
    servo = rospy.Rate(100)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start of the last cycle)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Build and publish the command message.  If we have a gravity
        # model, compute the gravity effort.  Leave other values alone.
        cmdmsg.header.stamp = servotime
        if   (t <= 0.25 * period):
            cmdmsg.position[joint] = center + speed * t
            cmdmsg.velocity[joint] =          speed
        elif (t <= 0.75 * period):
            cmdmsg.position[joint] = center - speed * (t - 0.5*period)
            cmdmsg.velocity[joint] =        - speed
        else:
            cmdmsg.position[joint] = center + speed * (t - period)
            cmdmsg.velocity[joint] =        + speed
        if callable(gravity):
            cmdmsg.effort = gravity(cmdmsg.position);
        pub.publish(cmdmsg)

        # Shift the (period's) start time after a full period
        if (t >= period):
            starttime = starttime + rospy.Duration.from_sec(period)

        # Wait for the next turn.
        servo.sleep()


def blocking_squarewave(pub, cmdmsg, joint, amplitude, period, gravity=None):
    """Triangle Wave
    Move a single joint according to a triangle wave of given
    amplitude and period.  Use a gravity model if available.
    """
    # Report
    rospy.loginfo('Running triangle wave on jnt %d, amp %f, period %f ...' %
                  (joint, amplitude, period))

    # Make sure we have a positive period.
    period = max(period, 0.01)

    # Remember the center and pre-compute the speed.
    center = cmdmsg.position[joint]
    speed  = 4.0 * amplitude / period

    # Use a 100Hz servo to send the commands.
    servo = rospy.Rate(100)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start of the last cycle)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Build and publish the command message.  If we have a gravity
        # model, compute the gravity effort.  Leave other values alone.
        cmdmsg.header.stamp = servotime
        if   (t <= 0.5 * period):
            cmdmsg.position[joint] = center + amplitude / 2.0
            cmdmsg.velocity[joint] = 0
        else:
            cmdmsg.position[joint] = center - amplitude / 2.0
            cmdmsg.velocity[joint] = 0
        if callable(gravity):
            cmdmsg.effort = gravity(cmdmsg.position);
        pub.publish(cmdmsg)

        # Shift the (period's) start time after a full period
        if (t >= period):
            starttime = starttime + rospy.Duration.from_sec(period)

        # Wait for the next turn.
        servo.sleep()


