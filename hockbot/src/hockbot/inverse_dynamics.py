import numpy as np

from sensor_msgs.msg import JointState


def plan_trajectory(starting_state, ending_state, total_time=0.8):
    """Pass in a commanded starting state and ending state,
    and this will compute a spline to execute that motion,
    as well as filling in the effort needed based off
    of the dynamics model of the SCARA robot."""
    dt = 0.01
    wall_t = 0
    splines = compute_spline(starting_state, ending_state)
    num_joints = len(starting_state.position)
    joint_states = []
    while wall_t <= total_time:
        t = wall_t / total_time
        jstate = JointState()
        jstate.name = starting_state.name
        jstate.position = [0]*num_joints
        jstate.velocity = [0]*num_joints
        jstate.effort = [0]*num_joints
        for i in range(num_joints):
            jstate.position[i] = splines[i](t)
            jstate.velocity[i] = splines[i].deriv()(t)
            # Use as placeholder for acceleration
            jstate.effort[i] = splines[i].deriv().deriv()(t)
        # Calculate torque
        print(jstate)
        jstate.effort = calculate_dynamics(
            jstate.position,
            jstate.velocity,
            jstate.effort)
        joint_states.append(jstate)
        wall_t += dt
    return joint_states


def compute_spline(starting_state, ending_state):
    joint_splines = [
        compute_1d_spline(s, e)
        for s, e in zip(
            starting_state.position,
            ending_state.position)]
    return joint_splines


def compute_1d_spline(starting_pos, ending_pos):
    start_end_constraints = np.array(
        [starting_pos,
         0,  # starting velocity
         0,  # starting acceleration
         ending_pos,
         0,  # ending velocity
         0,  # ending acceleration
         ], dtype=np.float32)
    X = np.array(
        [[1, 0, 0, 0,  0,  0],
         [0, 1, 0, 0,  0,  0],
         [0, 0, 1, 0,  0,  0],
         [1, 1, 1, 1,  1,  1],
         [0, 1, 2, 3,  4,  5],
         [0, 0, 2, 6, 12, 20]], dtype=np.float32)
    coeff = np.linalg.solve(X, start_end_constraints)
    print(coeff)
    # if not np.allclose(np.dot(X, coeff), start_end_constraints):
    #     raise ValueError('Unable to fit spline')
    return np.polynomial.polynomial.Polynomial(coeff)


def calculate_dynamics(pos_vector, vel_vector, accel_vector):
    """Calculate the effort vector for a custom 2-DoF SCARA."""
    m1 = 1.5  # kg weight of first body
    r1 = 0.35  # distance to cg
    d1 = 0.5   # full link length
    i1 = (1./8)*m1*d1**2

    m2 = 1.2
    r2 = 0.35
    i2 = (1./8)*m2*0.5**2

    A = i1 + m1*r1**2
    B = i2 + m2*r2**2

    # Simple friction model only looking at j1 torque
    friction = 0  # np.tanh(vel_vector[1]*10)*0.8
    # Flip joint torques because joints are flipped
    tau2 = B * (-accel_vector[0] + accel_vector[1]) + friction
    tau1 = A*accel_vector[0] - tau2

    return [tau1, tau2]
