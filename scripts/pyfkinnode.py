#!/usr/bin/env python
#
#   pyfkinnode.py
#
#   Compute the forward kinematics.
#
#   Subscribe: /joint_states      sensor_msgs/JointState
#   Publish:   /tippoint          geometry_msgs/PointStamped
#   Publish:   /tippose           geometry_msgs/PoseStamped
#
import rospy

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg   import JointState


#
#  Matrix Manipulation Utilities
#
def vec(x, y, z):
    return np.array([x, y, z])
def Rx(q):
    return Rotation.from_rotvec(q * vec(1, 0, 0))
def Ry(q):
    return Rotation.from_rotvec(q * vec(0, 1, 0))
def Rz(q):
    return Rotation.from_rotvec(q * vec(0, 0, 1))


#
#  FKin Node Class
#
class FKin:
    def __init__(self, jointnames):
        '''
        jointnames[] = [name of q0, name of q1, ...]
        Joints are labeled qn starting from n=0 at the base outward toward the tip
        '''
        self.jointnames = jointnames[:]

        # Constants for robot size
        self.W = 0.5 # Table width, meters
        self.L_1s = 0.2 # First SCARA arm length, meters
        self.L_2s = 0.2 # Second SCARA arm length, meters


        # Create publishers to send the tip position/pose.
        self.pointPublisher = rospy.Publisher("/tippoint", PointStamped)
        self.posePublisher  = rospy.Publisher("/tippose",  PoseStamped)


        # Create a subscriber to listen to joint_states.
        rospy.Subscriber("/hebiros/robot/feedback/joint_state", JointState, self.process)

    def process(self, msg):
        indices = {}
        for name in msg.name:
            if name in self.jointnames:
                indices[name] = msg.name.index(name)
        if len(indices) < len(self.jointnames):
            rospy.loginfo("Not all joints found in jointstate message")
            return

        # Start before Joint 0.
        x = vec(0, 0, 0)
        R = Rz(0)

        # W --> 0-
        x = x + vec(self.W/2.0, 0, 0)
        # 0- --> 0+
        R = R * Rz(msg.position[indices[self.jointnames[0]]])

        # 0+ --> 1-
        x = x + R.apply(vec(self.L_1s, 0, 0))
        # 1- --> 1+
        R = R * Rz(msg.position[indices[self.jointnames[1]]])

        # 1+ --> Tip
        x = x + R.apply(vec(0, self.L_2s, 0))

        # Publish the tip point.  Note that we declare the point as
        # given with respect to the world reference frame.
        point_msg = PointStamped()
        point_msg.header = msg.header;
        point_msg.header.frame_id = "world"

        point_msg.point.x = x[0]
        point_msg.point.y = x[1]
        point_msg.point.z = x[2]

        self.pointPublisher.publish(point_msg)

        # Publish the tip pose.  Note that we declare the pose as
        # given with respect to the world reference frame.
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position.x = x[0]
        pose_msg.pose.position.y = x[1]
        pose_msg.pose.position.z = x[2]

        quat = R.as_quat()
        pose_msg.pose.orientation.x = quat[0];
        pose_msg.pose.orientation.y = quat[1];
        pose_msg.pose.orientation.z = quat[2];
        pose_msg.pose.orientation.w = quat[3];

        self.posePublisher.publish(pose_msg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('fkinnode')

    # Instantiate the FKin object.
    fkin = FKin(['Chewbacca/q0', 'Chewbacca/q1'])

    # Spin until shutdown.
    rospy.loginfo("Fkin: Running...")
    rospy.spin()
