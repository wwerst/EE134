
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
