#!/usr/bin/env python3

import rospy
import numpy as np
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_matrix, translation_from_matrix, inverse_matrix, quaternion_from_matrix

def get_end_effector_transform():
    # Initialize the node
    rospy.init_node('end_effector_and_trackers_transform_node', anonymous=True)

    # Initialize MoveGroup for the UR10 arm
    group = MoveGroupCommander("ur_10")

    # Define the end effector link as "tool0"
    end_effector_link = "tool0"

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
    object_pose.position.z = position[2]
    object_pose.orientation = Quaternion(
        downward_orientation[0],
        downward_orientation[1],
        downward_orientation[2],
        downward_orientation[3]
    )

    return object_pose

if __name__ == '__main__':
    try:
        # Define the provided transformation matrices
        T_camera_to_ee = np.array([
            [-7.5639e-01, -6.5388e-01, -1.7812e-02, 3.2952e+02],
            [2.5999e-01, -3.2551e-01, 9.0909e-01, -5.7694e+01],
            [-6.0023e-01, 6.8300e-01, 4.1621e-01, 7.3475e+02],
            [0, 0, 0, 1.0000]
        ])

        T_camera_to_object = np.array([
            [2.5122e-01, -1.6444e-01, -9.5386e-01, 2.7461e+02],
            [-7.7373e-01, -6.2623e-01, -9.5822e-02, 3.0945e+02],
            [-5.8158e-01, 7.6210e-01, -2.8455e-01, 1.0252e+03],
            [0, 0, 0, 1.0000]
        ])

        # Convert translation components from mm to meters
        T_camera_to_ee[:3, 3] /= 1000.0
        T_camera_to_object[:3, 3] /= 1000.0

        # Get the end effector transformation in the base frame
        T_base_to_ee = get_end_effector_transform()

        # Calculate the object's pose in the base frame
        object_pose = calculate_object_pose(T_camera_to_object, T_camera_to_ee, T_base_to_ee)

        rospy.loginfo("Object pose in base frame: %s", object_pose)

        # Pass the `object_pose` to your robot's IK or pick-and-place function
    except rospy.ROSInterruptException:
        pass
