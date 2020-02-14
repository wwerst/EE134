import numpy as np

# Constants
SCARA_X_OFFSET = 0.0 # meters, indexed from bottom left corner
SCARA_Y_OFFSET = 0.0 # meters, indexed from bottom left corner

L1 = 0.5 # meters, length of the first arm
L2 = 0.5 # meters, length of the second arm

# Positional definitions for SCARA:
#
# Angles of q0, q1 defined WRT horizontal. 
#
# SCARA (0, 0)
#  |
#  v
# q0----(L1)------q1------(L2)----tip

def fkin_SCARA(q0, q1):
    '''
    Returns (x, y) position as a function of the joint angles q0, q1
    '''
    return (SCARA_X_OFFSET + L1*np.cos(q0) + L2*np.cos(q1), 
            SCARA_Y_OFFSET + L1*np.sin(q0) + L2*np.sin(q1))

def ikin_SCARA(x, y):
    '''
    Returns array of [(q0_1, q1_1), (q0_2, q1_2)] as a function of the (x, y) position given. Returns
    None if (x, y) is out of bounds or out of reach. 
    '''

    # Check if out of reach
    if (np.sqrt((x - SCARA_X_OFFSET)**2 + (y - SCARA_Y_OFFSET)**2) > L1 + L2):
        return None
    # Check if out of bounds
    else if (y < SCARA_Y_OFFSET):
        return None

    # Else, compute the branches
    x = x-SCARA_X_OFFSET
    y = y-SCARA_Y_OFFSET

    # Elbow right
    q0_1 = np.arctan2()
    # Elbow left