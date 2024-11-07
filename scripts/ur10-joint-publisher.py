#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

def radians_to_degrees(radians):
    # Convert radians to degrees
    return [math.degrees(rad) for rad in radians]

def swap_first_and_third(positions):
    # Swap the first and third positions
    if len(positions) >= 3:
        positions[0], positions[2] = positions[2], positions[0]
    return positions

def joint_states_callback(msg):
    # Get the current joint positions in radians
    joint_positions_radians = list(msg.position)

    # Swap first and third positions
    joint_positions_radians = swap_first_and_third(joint_positions_radians)

    # Convert the joint positions to degrees
    joint_positions_degrees = radians_to_degrees(joint_positions_radians)

    # Convert the list of degrees to a string
    joint_positions_str = ', '.join([f"{pos:.2f}" for pos in joint_positions_degrees])

    # Log the joint positions and publish them
    rospy.loginfo(f"Current joint positions in degrees: {joint_positions_str}")
    pub.publish(joint_positions_str)

def publish_actual_joint_positions():
    rospy.init_node('ur10_joint_publisher', anonymous=True)
    
    # Publisher to send joint positions in degrees as a string
    global pub
    pub = rospy.Publisher('/ur10_joint_positions_in_degrees', String, queue_size=10)
    
    # Subscribe to the /joint_states topic to get the actual joint positions
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    
    rospy.loginfo("UR10 Joint Position Publisher is running. Press Ctrl+C to stop.")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_actual_joint_positions()
    except rospy.ROSInterruptException:
        rospy.loginfo("UR10 Joint Position Publisher has been stopped.")
