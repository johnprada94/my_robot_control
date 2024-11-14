#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose


def calculate_ik(target_pose):
    # Initialize the node
    rospy.init_node('ik_calculation_node', anonymous=True)

    # Initialize MoveIt interface for the robot
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    # Initialize MoveGroup for the UR10 arm
    group = MoveGroupCommander("ur_10")

    # Set the target pose for the end effector
    group.set_pose_target(target_pose)

    # Compute the IK solution
    success, plan, planning_time, error_code = group.plan()

    # Check if planning was successful
    if success:
        # Extract the first and last joint values from the trajectory
        if plan and plan.joint_trajectory.points:  # If plan exists and has joint trajectory
            first_point = plan.joint_trajectory.points[0]
            last_point = plan.joint_trajectory.points[-1]

            # Store the first and last points into separate variables
            initial_positions = first_point.positions
            final_positions = last_point.positions

            # Get the joint names
            joint_names = plan.joint_trajectory.joint_names

            # Output for verification, printing joint names and positions
            rospy.loginfo("Initial joint positions:")
            for i, joint_name in enumerate(joint_names):
                rospy.loginfo("Joint %s: %f", joint_name, initial_positions[i])

            rospy.loginfo("Final joint positions:")
            for i, joint_name in enumerate(joint_names):
                rospy.loginfo("Joint %s: %f", joint_name, final_positions[i])

            return initial_positions, final_positions
        else:
            rospy.loginfo("No joint trajectory points found.")
            return None, None
    else:
        rospy.loginfo("IK solution not found or planning failed. Error Code: %d", error_code.val)
        return None, None

if __name__ == '__main__':
    try:
        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = 0.0
        target_pose.position.z = 0.5
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        # Calculate the IK for the specified pose
        initial_positions, final_positions = calculate_ik(target_pose)

        if initial_positions and final_positions:
            rospy.loginfo("Calculated initial and final joint positions:")
            rospy.loginfo("Initial: %s", initial_positions)
            rospy.loginfo("Final: %s", final_positions)
        else:
            rospy.loginfo("Failed to calculate IK.")

    except rospy.ROSInterruptException:
        pass
