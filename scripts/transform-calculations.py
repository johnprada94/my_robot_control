#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix, inverse_matrix

def get_transform_matrix():
    rospy.init_node('ur10_tf_listener')
    listener = tf.TransformListener()

    source_frame = "base_link"
    target_frame = "tool0"

    try:
        listener.waitForTransform(source_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
        
        # Construct transformation matrix from base to tool0
        T_base_to_tool0 = np.eye(4)
        T_base_to_tool0[0:3, 3] = trans
        T_base_to_tool0[0:3, 0:3] = quaternion_matrix(rot)[0:3, 0:3]

        rospy.loginfo(f"Transform Matrix (Base to Tool0):\n{T_base_to_tool0}")
        return T_base_to_tool0

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not get transform")
        return None

def compute_marker_in_base_frame(T_base_to_tool0, T_cam_to_marker):
    # Invert the camera-to-marker transform to get tool0-to-marker transform
    T_tool0_to_marker = inverse_matrix(T_cam_to_marker)
    
    # Calculate marker's pose in the robot's base frame
    T_base_to_marker = np.dot(T_base_to_tool0, T_tool0_to_marker)
    
    rospy.loginfo(f"Transform Matrix (Base to Marker):\n{T_base_to_marker}")
    return T_base_to_marker

if __name__ == "__main__":
    # Get the base-to-tool0 transformation
    T_base_to_tool0 = get_transform_matrix()
    
    # Marker transform matrix in the camera frame
    T_cam_to_marker = np.array([
        [3.8274e-01, -9.1847e-01, -9.9595e-02, 1.7562e+02],
        [3.2705e-01, 3.3880e-02, 9.4440e-01, -3.1659e+01],
        [-8.6403e-01, -3.9404e-01, 3.1335e-01, 5.6075e+02],
        [0, 0, 0, 1]
    ])
    
    if T_base_to_tool0 is not None:
        T_base_to_marker = compute_marker_in_base_frame(T_base_to_tool0, T_cam_to_marker)
