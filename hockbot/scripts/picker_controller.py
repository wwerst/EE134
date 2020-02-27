#!/usr/bin/env python
from threading import Lock
import numpy as np

import rospy
from hockbot.srv import (
    MovePickerCart,
    MovePickerCartResponse,
)
from sensor_msgs.msg import JointState

import actionlib
from hebiros.msg import (
    TrajectoryAction,
    TrajectoryGoal,
    WaypointMsg,
)
from hockbot.fourdof_kinematics import (
    FourDoFKinematics,
    SURFACE_PLAY,
    SURFACE_COLLECT,
)
from hockbot.inverse_dynamics import plan_trajectory
from hockbot.gravity_comp import joint_gravity_torque
from hockbot.utils import remap_joint_state


class PickerController(object):

    def __init__(self):
        self._hebi_commander = rospy.Publisher(
            '/hebiros/picker/command/joint_state',
            JointState,
            queue_size=1)

        self._cmd_pos_lock = Lock()
        self._cmd_pos = None
        self._cur_pos = rospy.wait_for_message(
            '/hebiros/picker/feedback/joint_state',
            JointState)
        self._pos_sub = rospy.Subscriber(
            '/hebiros/picker/feedback/joint_state',
            JointState,
            self._pos_cb)
        self._kin = FourDoFKinematics()
        self._cart_srv = rospy.Service(
            '~/move_picker_cart',
            MovePickerCart,
            self._move_cart_cb
            )
        self._timer_cb = rospy.Timer(
            rospy.Duration(0.01),
            self._pub_joint_cmd)

    def _pos_cb(self, msg):
        self._cur_pos = msg

    @property
    def fourdof_start_pos(self):
        with self._cmd_pos_lock:
            pos = self._cmd_pos
        if pos is None:
            pos = self._cur_pos
        pos_dict = {name: pos.position[i] for i, name in enumerate(pos.name)}
        return np.array([
            pos_dict['Picker/0'],
            pos_dict['Picker/1'],
            pos_dict['Picker/2'],
            pos_dict['Picker/4'],])

    def jointstate_fourdof_angles(self, angles):
        """Convert fourdof angle command to 5-DoF command."""
        joint_names = ['Picker/0', 'Picker/1', 'Picker/2', 'Picker/3', 'Picker/4']
        joint_pos = np.insert(angles, 3, 0)
        return JointState(
            name=joint_names,
            position=joint_pos,
            velocity=[0]*len(joint_names),
            effort=joint_gravity_torque(joint_pos))

    def _move_cart_cb(self, req):
        assert req.move_time > 0.2, 'move_time too small'
        if not req.cartesian_interp:
            raise NotImplementedError('Joint movement not supported')
        else:
            traj = self.plan_cartesian(np.array([req.x, req.y, req.z]), req.tool_vertical, req.move_time)
        self._send_hebi_command_traj(traj, 0.01)
        return True

    def _send_hebi_command_traj(self, joint_states, dt):
        """Sends a trajectory to robot by streaming to
        command topic."""
        rate = rospy.Rate(1.0/dt)
        for j in joint_states:
            with self._cmd_pos_lock:
                self._cmd_pos = j
            rate.sleep()

    def _pub_joint_cmd(self, _):
        with self._cmd_pos_lock:
            cmd_pos = self._cmd_pos
        if cmd_pos is not None:
            cmd_pos = remap_joint_state(cmd_pos, ['Picker/0', 'Picker/1', 'Picker/2', 'Picker/3', 'Picker/4'])
            cmd_pos.effort = joint_gravity_torque(cmd_pos.position)
            self._hebi_commander.publish(cmd_pos)

    def plan_cartesian(self, goal_pos, eff_vertical, goal_time=5.0, dt=0.01):
        eff_surface = SURFACE_PLAY if eff_vertical else SURFACE_COLLECT
        start_pos = self._kin.fkin(*self.fourdof_start_pos)[0:3]
        def interp_pos(fraction):
            print('start_pos: {}'.format(start_pos))
            print('goal_pos: {}'.format(goal_pos))
            print()
            return start_pos + (goal_pos - start_pos)*fraction
        positions = []
        for frac in np.linspace(0.0, 1.0, goal_time//dt):
            smooth_frac = 2*frac**2 if frac < 0.5 else (-2*(frac-1)**2 + 1)
            step_pos = interp_pos(smooth_frac)
            print('Step_Pos: {}'.format(step_pos))
            solution_angles = self._kin.ikin(step_pos[0], step_pos[1], step_pos[2], eff_surface)

            jstate = self.jointstate_fourdof_angles(solution_angles)
            positions.append(jstate)
        return positions


def main():
    rospy.init_node('scara_controller')
    controller = PickerController()
    rospy.spin()


if __name__ == '__main__':
    main()
