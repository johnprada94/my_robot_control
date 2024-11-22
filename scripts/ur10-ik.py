#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState  # Import JointState message
from std_msgs.msg import String

# Initialize the MoveIt interfaces globally so they are loaded only once
robot = None
scene = None
group = None
joint_state_publisher = None  # Publisher for final joint positions

def initialize_moveit():
    global robot, scene, group, joint_state_publisher
    # Initialize MoveIt interface for the robot (only once)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("ur_10")

    # Initialize the publisher for sending joint states
    joint_state_publisher = rospy.Publisher('/final_joint_positions', JointState, queue_size=10)

def calculate_ik(target_pose):
    # Get the current joint values
    current_joint_positions = group.get_current_joint_values()

    # Set the initial guess for the IK solver to the current joint positions
    group.set_start_state_to_current_state()

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

            # Publish the final joint positions
            publish_joint_positions(final_positions, joint_names)

            return initial_positions, final_positions
        else:
            rospy.loginfo("No joint trajectory points found.")
            return None, None
    else:
        rospy.loginfo("IK solution not found or planning failed. Error Code: %d", error_code.val)
        return None, None

def publish_joint_positions(final_positions, joint_names):
    # Create and populate the JointState message
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = joint_names
    joint_state_msg.position = final_positions

    # Publish the joint state message
    joint_state_publisher.publish(joint_state_msg)
    rospy.loginfo("Published final joint positions.")

def object_pose_callback(msg):
    # Parse the incoming object pose message
    target_pose = Pose()
    target_pose.position.x = msg.position.x
    target_pose.position.y = msg.position.y
    target_pose.position.z = msg.position.z
    target_pose.orientation = msg.orientation

    # Calculate the IK for the received object pose
    initial_positions, final_positions = calculate_ik(target_pose)

    if initial_positions and final_positions:
        rospy.loginfo("Calculated initial and final joint positions:")
        rospy.loginfo("Initial: %s", initial_positions)
        rospy.loginfo("Final: %s", final_positions)
    else:
        rospy.loginfo("Failed to calculate IK.")

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('ik_calculation_node', anonymous=True)

        # Initialize MoveIt only once
        initialize_moveit()

        # Subscribe to the /object_pose topic
        rospy.Subscriber("/object_pose", Pose, object_pose_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
