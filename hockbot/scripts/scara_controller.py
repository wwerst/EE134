#!/usr/bin/env python
import numpy as np

import rospy
from hockbot.srv import (
    MoveCart,
    MoveJoint,
)
from sensor_msgs.msg import JointState

import actionlib
from hebiros.msg import (
    TrajectoryAction,
    TrajectoryGoal,
    WaypointMsg,
)
from hockbot.scara_kinematics import (
    ikin,
    fkin,
    jacobian,
)
from hockbot.inverse_dynamics import plan_trajectory


class ScaraController(object):

    def __init__(self):
        self._hebi_commander = rospy.Publisher(
            '/hebiros/scara/command/joint_state',
            JointState,
            queue_size=1)

        self._cur_pos = rospy.wait_for_message(
            '/hebiros/scara/feedback/joint_state',
            JointState)
        self._pos_sub = rospy.Subscriber(
            '/hebiros/scara/feedback/joint_state',
            JointState,
            self._pos_cb)
        self._joint_srv = rospy.Service(
            '~/move_joint',
            MoveJoint,
            self._move_joint_cb
            )
        self._cart_srv = rospy.Service(
            '~/move_cart',
            MoveCart,
            self._move_cart_cb
            )

    def _pos_cb(self, msg):
        self._cur_pos = msg

    def _move_cart_cb(self, req):
        assert req.move_time > 0.2, 'move_time too small'
        if req.joint_space_interp:
            theta_1, theta_2 = ikin(req.x, req.y)[0]
            goal_state = JointState(
                name=['Scara/0', 'Scara/1'],
                position=[theta_1, theta_2])
            traj = plan_trajectory(self._cur_pos, goal_state, req.move_time)
        else:
            traj = self.plan_cartesian(np.array([req.x, req.y]), req.move_time)
        self._send_hebi_command_traj(traj, 0.01)
        return True

    def _move_joint_cb(self, req):
        assert req.move_time > 0.2, 'move_time too small'
        goal_state = JointState(
            name=['Scara/0', 'Scara/1'],
            position=[req.theta1, req.theta2])
        traj = plan_trajectory(self._cur_pos, goal_state, req.move_time)
        self._send_hebi_command_traj(traj, 0.01)
        return True

    def _send_hebi_command_traj(self, joint_states, dt):
        """Sends a trajectory to robot by streaming to
        command topic."""
        rate = rospy.Rate(1.0/dt)
        for j in joint_states:
            self._hebi_commander.publish(j)
            rate.sleep()

    def _send_hebi_traj_goal(self, joint_states, dt):
        """Sends a trajectory to robot by compiling trajectory
        and sending as action server goal."""
        goal = TrajectoryGoal(waypoints=[], times=[])
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
        client = actionlib.SimpleActionClient('/hebiros/scara/trajectory', TrajectoryAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()

    def plan_cartesian(self, goal_pos, goal_time=5.0, dt=0.01):
        cur_angles = self._cur_pos
        cur_angles = np.array([cur_angles.position[0], cur_angles.position[1]])
        start_pos = fkin(*cur_angles)
        def interp_pos(fraction):
            print('start_pos: {}'.format(start_pos))
            print('goal_pos: {}'.format(goal_pos))
            print()
            return start_pos + (goal_pos - start_pos)*fraction
        positions = []
        for frac in np.linspace(0.0, 1.0, goal_time//dt):
            step_pos = interp_pos(frac)
            print('Step_Pos: {}'.format(step_pos))
            solution_angles = ikin(*step_pos)
            print(solution_angles)
            print(cur_angles)
            angle_delta = np.linalg.norm(
                solution_angles - cur_angles, axis=1)
            # HACK(WHW):
            min_index = 0 if angle_delta[0] < angle_delta[1] else 1
            cur_angles = solution_angles[min_index]
            jstate = JointState()
            jstate.name = ['Scara/0', 'Scara/1']
            jstate.position = cur_angles
            jstate.velocity = [0, 0]
            jstate.effort = [0, 0]
            positions.append(jstate)
        return positions


def main():
    rospy.init_node('scara_controller')
    controller = ScaraController()
    rospy.spin()


if __name__ == '__main__':
    main()
