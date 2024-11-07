#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import euler_from_quaternion
import numpy as np

def get_transform_matrix():
    # Initialize the ROS node
    rospy.init_node('ur10_tf_listener')

    # Initialize the TF listener
    listener = tf.TransformListener()

    # Define the frames you want the transform from and to
    source_frame = "base_link"  # Change this if you want another frame
    target_frame = "tool0"   # End effector frame (you can change this)

    try:
        # Wait for the transform to be available
        listener.waitForTransform(source_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))

        # Get the transform from source to target
        (trans, rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))

        # Convert the quaternion rotation into Euler angles
        roll, pitch, yaw = euler_from_quaternion(rot)

        # Print the translation and rotation
        rospy.loginfo(f"Translation: {trans}")
        rospy.loginfo(f"Rotation (Euler angles): roll={roll}, pitch={pitch}, yaw={yaw}")

        # Construct a 4x4 transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 3] = trans
        transform_matrix[0:3, 0:3] = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]

        rospy.loginfo(f"Transform Matrix:\n{transform_matrix}")
        return transform_matrix

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not get transform")
        return None

if __name__ == "__main__":
    transform_matrix = get_transform_matrix()
    if transform_matrix is not None:
        rospy.loginfo("Transformation matrix retrieved successfully.")
