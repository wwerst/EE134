import numpy as np

j4_param = -0.18
j2_param = -3.8
j1_param = -5.0


def joint_gravity_torque(q):
    """Takes in a numpy array of joint angles and returns
    the resultant gravity torque."""

    j1_angle = -q[1]
    j2_angle = j1_angle + q[2]
    j4_angle = j2_angle - q[4]

    j4_torque = j4_param * np.sin(j4_angle)
    j2_torque = j4_torque + j2_param * np.sin(j2_angle)
    j1_torque = j2_torque + j1_param * np.sin(j1_angle)

    return np.array([
        0.0,
        -j1_torque,
        j2_torque,
        0.0,
        -j4_torque])
