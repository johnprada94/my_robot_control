#!/usr/bin/env python3

import rospy
import tf
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose

def get_end_effector_transform():
    # Initialize the node
    rospy.init_node('end_effector_transform_node', anonymous=True)

    # Initialize MoveGroup for the UR10 arm
    group = MoveGroupCommander("ur_10")

    # Define the end effector link as "tool0"
    end_effector_link = "tool0"

    # Get the current pose of the end effector (tool0) in the base frame
    end_effector_pose = group.get_current_pose(end_effector_link)

    rospy.loginfo("End effector pose (in base frame): %s", end_effector_pose)

    # Convert PoseStamped to Pose for IK calculation
    target_pose = Pose()
    target_pose.position = end_effector_pose.pose.position
    target_pose.orientation = end_effector_pose.pose.orientation

    return target_pose

if __name__ == '__main__':
    try:
        target_pose = get_end_effector_transform()

        # Now, pass this target_pose to the IK script
        rospy.loginfo("End effector target pose for IK: %s", target_pose)
    except rospy.ROSInterruptException:
        pass
