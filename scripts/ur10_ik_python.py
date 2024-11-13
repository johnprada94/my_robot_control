#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander import PlanningSceneInterface

def main():
    # Initialize the moveit_commander and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_ik_python', anonymous=True)

    # Instantiate RobotCommander, PlanningSceneInterface, and MoveGroupCommander
    robot = moveit_commander.robot.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # Replace with your actual group name (e.g., 'arm')
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get the current pose of the robot's end effector
    end_effector_link = move_group.get_end_effector_link()
    print("Current pose of the end effector:")
    print(move_group.get_current_pose(end_effector_link))

    # Define a target pose (example: a 6D pose with position and orientation)
    target_pose = Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.2
    target_pose.position.z = 0.5
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    # Set the target pose to the MoveGroupCommander
    move_group.set_pose_target(target_pose)

    # Compute the inverse kinematics solution
    plan = move_group.go(wait=True)  # Execute the plan

    # Check if the planning was successful
    if plan:
        print("IK found and executed successfully.")
    else:
        print("IK did not find a solution.")
    
    # Optionally, get the joint values and print them
    joint_values = move_group.get_current_joint_values()
    print("Joint values:")
    print(joint_values)

    # Shutdown moveit_commander when done
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
