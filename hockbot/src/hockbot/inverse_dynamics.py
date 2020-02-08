from sensors_msgs import JointState


def plan_trajectory(starting_state, ending_state):
    """Pass in a commanded starting state and ending state,
    and this will compute a spline to execute that motion,
    as well as filling in the effort needed based off
    of the dynamics model of the SCARA robot."""
    joint_states = []
    return joint_states
