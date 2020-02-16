import numpy as np

# Constants
SCARA_X_OFFSET = 0.0 # meters, indexed from bottom left corner
SCARA_Y_OFFSET = 0.0 # meters, indexed from bottom left corner

r1 = 0.5 # meters, length of the first arm
r2 = 0.5 # meters, length of the second arm

# Positional definitions for SCARA:
#
# Angles of theta_1, theta_2 defined WRT horizontal. 
#
# SCARA (0, 0)
#  |
#  v
# theta_1----(r1)------theta_2------(r2)----tip


def fkin(theta_1, theta_2):
    '''
    Returns (x, y) position as a function of the joint angles q0, q1
    '''
    return np.array([
        SCARA_X_OFFSET +r1*np.cos(theta_1) + r2*np.cos(-theta_2),
        SCARA_Y_OFFSET +r1*np.sin(theta_1) + r2*np.sin(-theta_2)])


def ikin(x, y):
    '''
    Returns array of [(theta_1_1, theta_1_2), (theta_2_1, theta_2_2)] as a
    function of the (x, y) position given. Returns
    None if (x, y) is out of bounds or out of reach.
    '''

    # Else, compute the branches
    x = x - SCARA_X_OFFSET
    y = y - SCARA_Y_OFFSET

    # Check if out of reach
    if (np.sqrt((x - SCARA_X_OFFSET)**2 + (y - SCARA_Y_OFFSET)**2) > r1 + r2):
        return None
    # Check if out of bounds
    if (y < SCARA_Y_OFFSET):
        return None

    # angle from base to desired point
    theta_t = np.arctan2(y, x)
    
    # calculate the angle at the elbow using law  of cosines
    theta_1_2 = np.arccos((x**2 + y**2 - r1**2 -r2**2)/(2*r1*r2))   

    # difference between angles 
    theta_1_1 = theta_t - np.arctan2((r1*np.sin(theta_1_2)), (r1 + r2 * np.cos(theta_1_2)))

    # calculate the second set of angles
    theta_2_2 = -1 * theta_1_2

    theta_2_1 = theta_t + (theta_t - theta_1_1)

    return np.array(
        [[theta_1_1, -(theta_1_1+theta_1_2)],
         [theta_2_1, -(theta_2_1+theta_2_2)]])


def jacobian(thetas):
    return np.array(
        [[-r1*np.sin(thetas[0]), -r2*np.sin(thetas[1])],
         [r1*np.cos(thetas[0]), r2*np.cos(thetas[1])]])

