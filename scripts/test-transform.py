#!/usr/bin/env python3

import rospy
import tf
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

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

    # Initialize the TF listener to get the transform from the base frame
    listener = tf.TransformListener()

    try:
        # Wait for the transform to be available
        listener.waitForTransform("base_link", end_effector_link, rospy.Time(0), rospy.Duration(4.0))

        # Get the transform from base_link to the end effector
        (trans, rot) = listener.lookupTransform("base_link", end_effector_link, rospy.Time(0))

        # Print the transformation in terms of translation and rotation
        rospy.loginfo("Transform from base_link to end effector:")
        rospy.loginfo("Translation: %s", trans)
        rospy.loginfo("Rotation: %s", rot)

        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not get transform from base_link to end effector")
        return None, None

if __name__ == '__main__':
    try:
        trans, rot = get_end_effector_transform()
        if trans and rot:
            rospy.loginfo("End effector transform successfully calculated.")
        else:
            rospy.loginfo("Failed to calculate end effector transform.")
    except rospy.ROSInterruptException:
        pass
