#!/usr/bin/env python3

import rospy
import numpy as np
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_matrix, translation_from_matrix, inverse_matrix, quaternion_from_matrix
from std_msgs.msg import String

# Initialize MoveGroupCommander once, outside of the function
group = None
end_effector_link = "tool0"
object_pose_pub = None  # Declare the publisher variable

def initialize_move_group():
    global group
    if group is None:
        group = MoveGroupCommander("ur_10")  # Initialize once when the node starts

def get_end_effector_transform():
    # Ensure the MoveGroupCommander is initialized
    initialize_move_group()

    # Get the current pose of the end effector (tool0) in the base frame
    end_effector_pose = group.get_current_pose(end_effector_link)

    rospy.loginfo("End effector pose (in base frame): %s", end_effector_pose)

    # Convert PoseStamped to 4x4 transformation matrix
    translation = [
        end_effector_pose.pose.position.x,
        end_effector_pose.pose.position.y,
        end_effector_pose.pose.position.z
    ]
    quaternion = [
        end_effector_pose.pose.orientation.x,
        end_effector_pose.pose.orientation.y,
        end_effector_pose.pose.orientation.z,
        end_effector_pose.pose.orientation.w
    ]
    rotation_matrix = quaternion_matrix(quaternion)
    T_base_to_ee = np.eye(4)
    T_base_to_ee[:3, :3] = rotation_matrix[:3, :3]
    T_base_to_ee[:3, 3] = translation

    return T_base_to_ee

def calculate_object_pose(T_camera_to_object, T_camera_to_ee, T_base_to_ee):
    # Compute T_ee_to_object
    T_camera_to_ee_inv = inverse_matrix(T_camera_to_ee)
    T_ee_to_object = np.dot(T_camera_to_ee_inv, T_camera_to_object)

    # Compute T_base_to_object
    T_base_to_object = np.dot(T_base_to_ee, T_ee_to_object)

    # Extract translation (position)
    position = translation_from_matrix(T_base_to_object)

    # Set orientation to look downward (Z-axis pointing down)
    downward_orientation = quaternion_from_matrix([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])

    # Create Pose object for the object in the base frame
    object_pose = Pose()
    object_pose.position.x = position[0]
    object_pose.position.y = position[1]
    object_pose.position.z = position[2]+0.06  # Add 6 cm to the Z position
    object_pose.orientation = Quaternion(
        downward_orientation[0],
        downward_orientation[1],
        downward_orientation[2],
        downward_orientation[3]
    )

    return object_pose

def parse_transform_data(data):
    """
    Parse the incoming transformation string and extract the Robot and Object transformation matrices
    using delimiters to split the data.
    """

    # Remove unnecessary newlines and extra spaces
    cleaned_data = data.replace("\n", " ").strip()

    # Split data based on the "Robot:" and "Object:" labels
    robot_data = cleaned_data.split("Robot:")[1].split("Object:")[0].strip()
    object_data = cleaned_data.split("Object:")[1].strip()

    # Convert the data strings into matrices (4x4), skipping invalid entries
    def convert_to_float_safe(data_str):
        try:
            return float(data_str)
        except ValueError:
            # If conversion fails, return None to skip this value
            return None

    T_camera_to_ee = []
    for i in range(0, len(robot_data.split()), 4):
        row = []
        for value in robot_data.split()[i:i+4]:
            converted_value = convert_to_float_safe(value)
            if converted_value is not None:
                row.append(converted_value)
        if len(row) == 4:  # Ensure the row is complete before adding
            T_camera_to_ee.append(row)

    T_camera_to_object = []
    for i in range(0, len(object_data.split()), 4):
        row = []
        for value in object_data.split()[i:i+4]:
            converted_value = convert_to_float_safe(value)
            if converted_value is not None:
                row.append(converted_value)
        if len(row) == 4:  # Ensure the row is complete before adding
            T_camera_to_object.append(row)

    # If either matrix is not 4x4, skip the data and return empty matrices
    try:
        T_camera_to_ee = np.array(T_camera_to_ee)
        T_camera_to_object = np.array(T_camera_to_object)

        # Check if both matrices are 4x4, otherwise raise an error
        if T_camera_to_ee.shape != (4, 4) or T_camera_to_object.shape != (4, 4):
            raise ValueError(f"Invalid shape: T_camera_to_ee={T_camera_to_ee.shape}, T_camera_to_object={T_camera_to_object.shape}")

    except ValueError as e:
        rospy.logwarn("Skipping invalid transform data due to shape mismatch: %s", e)
        return None, None  # Return None to indicate the data is invalid

    return T_camera_to_ee, T_camera_to_object


def transform_callback(msg):
    # Parse the transformation data from the incoming message
    T_camera_to_ee, T_camera_to_object = parse_transform_data(msg.data)

    # If the data is invalid, just return and wait for the next one
    if T_camera_to_ee is None or T_camera_to_object is None:
        return

    # Convert translation components from mm to meters
    T_camera_to_ee[:3, 3] /= 1000.0
    T_camera_to_object[:3, 3] /= 1000.0

    # Get the end effector transformation in the base frame
    T_base_to_ee = get_end_effector_transform()

    # Calculate the object's pose in the base frame
    object_pose = calculate_object_pose(T_camera_to_object, T_camera_to_ee, T_base_to_ee)

    rospy.loginfo("Object pose in base frame: %s", object_pose)

    # Publish the object pose
    object_pose_pub.publish(object_pose)

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('end_effector_and_trackers_transform_node', anonymous=True)

        # Create the publisher for the object pose
        object_pose_pub = rospy.Publisher('/object_pose', Pose, queue_size=10)

        # Subscribe to the topic publishing transformation data
        rospy.Subscriber("/tcp_server_transforms", String, transform_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass