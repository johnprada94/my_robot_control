#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from std_msgs.msg import String
from tf.transformations import euler_from_matrix, quaternion_matrix, inverse_matrix

def get_cartesian_coordinates(T):
    # Extract position (translation)
    position = T[0:3, 3]
    
    # Extract orientation (rotation) as Euler angles
    rotation_matrix = T[0:3, 0:3]
    euler_angles = euler_from_matrix(rotation_matrix)  # Returns roll, pitch, yaw
    
    return position, euler_angles

def get_transform_matrix():
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
    T_tool0_to_marker = inverse_matrix(T_cam_to_marker)
    T_base_to_marker = np.dot(T_base_to_tool0, T_tool0_to_marker)
    return T_base_to_marker

def compute_object_in_base_frame(T_base_to_marker, T_cam_to_object, T_cam_to_marker):
    T_marker_to_camera = inverse_matrix(T_cam_to_marker)
    T_base_to_camera = np.dot(T_base_to_marker, T_marker_to_camera)
    T_base_to_object = np.dot(T_base_to_camera, T_cam_to_object)
    return T_base_to_object

def tcp_server_transform_callback(msg):
    import re

    # Clean up the message and ensure data is in a single line
    single_line_data = " ".join(msg.data.split())
    rospy.loginfo(f"Received data (single line): {single_line_data}")

    robot_match = re.search(r'Robot:\s*([-\d.e+\s]+?)(?=Object:|$)', single_line_data)
    object_match = re.search(r'Object:\s*([-\d.e+\s]+?)(?=Robot:|$)', single_line_data)

    if robot_match:
        robot_matrix_str = robot_match.group(1)
        robot_matrix_values = list(map(float, robot_matrix_str.split()))
        if len(robot_matrix_values) == 16:
            T_cam_to_robot = np.array(robot_matrix_values).reshape(4, 4)
            rospy.loginfo(f"Robot Matrix:\n{T_cam_to_robot}")
        else:
            rospy.logwarn("Robot matrix does not contain exactly 16 values.")
    else:
        rospy.logwarn("No Robot matrix found in the message.")
    
    if object_match:
        object_matrix_str = object_match.group(1)
        object_matrix_values = list(map(float, object_matrix_str.split()))
        if len(object_matrix_values) == 16:
            T_cam_to_object = np.array(object_matrix_values).reshape(4, 4)
            rospy.loginfo(f"Object Matrix:\n{T_cam_to_object}")
        else:
            rospy.logwarn("Object matrix does not contain exactly 16 values.")
    else:
        rospy.logwarn("No Object matrix found in the message.")

    T_base_to_tool0 = get_transform_matrix()
    if T_base_to_tool0 is not None:
        T_base_to_marker = compute_marker_in_base_frame(T_base_to_tool0, T_cam_to_robot)
        T_base_to_object = compute_object_in_base_frame(T_base_to_marker, T_cam_to_object, T_cam_to_robot)
        
        # Calculate Cartesian coordinates for the marker and object
        marker_position, marker_orientation = get_cartesian_coordinates(T_base_to_marker)
        object_position, object_orientation = get_cartesian_coordinates(T_base_to_object)

        rospy.loginfo(f"Marker Position (Base frame): {marker_position}")
        rospy.loginfo(f"Marker Orientation (Base frame): {marker_orientation}")
        rospy.loginfo(f"Object Position (Base frame): {object_position}")
        rospy.loginfo(f"Object Orientation (Base frame): {object_orientation}")

def listen_to_tcp_server_transforms():
    rospy.loginfo("Listening to /tcp_server_transforms...")
    rospy.Subscriber('/tcp_server_transforms', String, tcp_server_transform_callback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ur10_tf_listener')
    listen_to_tcp_server_transforms()
    
