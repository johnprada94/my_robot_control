#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_matrix, inverse_matrix

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
    # Invert the camera-to-marker transform to get tool0-to-marker transform
    T_tool0_to_marker = inverse_matrix(T_cam_to_marker)
    
    # Calculate marker's pose in the robot's base frame
    T_base_to_marker = np.dot(T_base_to_tool0, T_tool0_to_marker)
    
    # rospy.loginfo(f"Transform Matrix (Base to Marker):\n{T_base_to_marker}")
    return T_base_to_marker

def compute_object_in_base_frame(T_base_to_marker, T_cam_to_object, T_cam_to_marker):
    # Calculate T_base_to_camera using T_base_to_marker and the inverse of T_cam_to_marker
    T_marker_to_camera = inverse_matrix(T_cam_to_marker)
    T_base_to_camera = np.dot(T_base_to_marker, T_marker_to_camera)
    
    # Calculate object's pose in the robot's base frame
    T_base_to_object = np.dot(T_base_to_camera, T_cam_to_object)
    
    # rospy.loginfo(f"Transform Matrix (Base to Object):\n{T_base_to_object}")
    return T_base_to_object

# Callback function to handle incoming String messages
def tcp_server_transform_callback(msg):
    import numpy as np
    import re

    # Clean up the message and ensure data is in a single line
    single_line_data = " ".join(msg.data.split())

    rospy.loginfo(f"Received data (single line): {single_line_data}")  # Debugging log

    # Capture only the first instance of Robot and Object matrices using non-greedy matching
    robot_match = re.search(r'Robot:\s*([-\d.e+\s]+?)(?=Object:|$)', single_line_data)
    object_match = re.search(r'Object:\s*([-\d.e+\s]+?)(?=Robot:|$)', single_line_data)

    # Process the first Robot matrix found
    if robot_match:
        robot_matrix_str = robot_match.group(1)
        robot_matrix_values = list(map(float, robot_matrix_str.split()))
        if len(robot_matrix_values) == 16:  # Ensure we have exactly 16 values
            T_cam_to_robot = np.array(robot_matrix_values).reshape(4, 4)
            rospy.loginfo(f"Robot Matrix:\n{T_cam_to_robot}")
        else:
            rospy.logwarn("Robot matrix does not contain exactly 16 values.")
    else:
        rospy.logwarn("No Robot matrix found in the message.")
    
    # Process the first Object matrix found
    if object_match:
        object_matrix_str = object_match.group(1)
        object_matrix_values = list(map(float, object_matrix_str.split()))
        if len(object_matrix_values) == 16:  # Ensure we have exactly 16 values
            T_cam_to_object = np.array(object_matrix_values).reshape(4, 4)
            rospy.loginfo(f"Object Matrix:\n{T_cam_to_object}")
        else:
            rospy.logwarn("Object matrix does not contain exactly 16 values.")
    else:
        rospy.logwarn("No Object matrix found in the message.")
    # Get the base-to-tool0 transformation
    T_base_to_tool0 = get_transform_matrix()
    if T_base_to_tool0 is not None:
        # Step 1: Compute base-to-RobotMarker transformation
        T_base_to_marker = compute_marker_in_base_frame(T_base_to_tool0, T_cam_to_robot)
        
        # Step 2: Compute base-to-object transformation using T_base_to_marker
        T_base_to_object = compute_object_in_base_frame(T_base_to_marker, T_cam_to_object,T_cam_to_robot)
        
        rospy.loginfo(f"Transform Matrix (Base to Robot-Marker):\n{T_base_to_marker}")
        rospy.loginfo(f"Transform Matrix (Base to Object):\n{T_base_to_object}")






def listen_to_tcp_server_transforms():
    rospy.loginfo("Listening to /tcp_server_transforms...")
    rospy.Subscriber('/tcp_server_transforms', String, tcp_server_transform_callback)
    rospy.spin()  # Keeps the node running and listening to the topic

if __name__ == "__main__":
    rospy.init_node('ur10_tf_listener')  # Ensure the node is initialized first
    
    # Start the listener for /tcp_server_transforms topic
    listen_to_tcp_server_transforms()