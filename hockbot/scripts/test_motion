#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

import actionlib
from hebiros.msg import (
    TrajectoryAction,
    TrajectoryActionGoal,
    WaypointMsg,
)
from hockbot.inverse_dynamics import plan_trajectory


def main():
    rospy.init_node('test_motion')
    hebi_commander = rospy.Publisher(
        '/hebiros/robot/command/joint_state',
        JointState,
        queue_size=1)
    position_loop = [
        (JointState(name=['Scara/0', 'Scara/1'], position=[0.2, -2.8]), 1.5),
        (JointState(name=['Scara/0', 'Scara/1'], position=[2.8, -0.2]), 1.5),
        (JointState(name=['Scara/0', 'Scara/1'], position=[0.2, -2.8]), 1.5),
        (JointState(name=['Scara/0', 'Scara/1'], position=[2.8, -0.2]), 1.5),
    ] * 10
    position_loop.append(position_loop[0])
    start_state = position_loop[0][0]
    plan_execute_motion(start_state, start_state, total_time=3.0)
    for i in range(len(position_loop) - 1):
        plan_execute_motion(position_loop[i][0], position_loop[i + 1][0], total_time=position_loop[i+1][1]*1.0)
        rospy.sleep(0.2)
        if rospy.is_shutdown():
            break


def plan_execute_motion(start_state, end_state, total_time=1.0):
    hebi_commander = rospy.Publisher(
        '/hebiros/robot/command/joint_state',
        JointState,
        queue_size=1)
    joints = plan_trajectory(start_state, end_state, total_time=total_time)
    rate = rospy.Rate(100)
    for j in joints:
        hebi_commander.publish(j)
        rate.sleep()


def send_hebi_action_msg(joint_states, dt):
    goal = TrajectoryActionGoal(waypoints=[], times=[])
    t = dt
    for j in joint_states:
        waypoint_msg = WaypointMsg(
            names=j.name,
            positions=j.position,
            velocities=j.velocity,
            accelerations=j.effort)
        goal.waypoints.append(waypoint_msg)
        goal.times.append(t)
        t += dt
    client = actionlib.SimpleActionClient('/hebiros/robot/trajectory', TrajectoryAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    main()
