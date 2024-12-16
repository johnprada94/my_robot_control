#!/usr/bin/env python3

import sys
import math
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from sensor_msgs.msg import JointState  # Import JointState message

# Correct joint order for the UR10 robot
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_NAMES_HOME = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

HOME_POSITION = [-1.5743916670428675, -1.1931417624102991, -0.038750473652974904, 
                 -1.4932034651385706, 1.53656005859375, 0.007116382941603661]

class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("ur10_action")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]

        # Subscriber to listen for joint positions from the /final_joint_positions topic
        self.joint_positions = None  # Initially set to None until data is received
        self.last_trajectory = None  # To store the last trajectory
        rospy.Subscriber("/final_joint_positions", JointState, self.joint_positions_callback)

    def joint_positions_callback(self, msg):
        """Callback function to update joint positions from the /final_joint_positions topic"""
        try:
            joint_positions = msg.position
            if len(joint_positions) == len(JOINT_NAMES):
                self.joint_positions = joint_positions
                rospy.loginfo("Received joint positions (radians): {}".format(self.joint_positions))
            else:
                rospy.logwarn("Received incorrect number of joint positions: {}".format(len(joint_positions)))
        except ValueError:
            rospy.logerr("Failed to extract joint positions from JointState message.")

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""
        if self.joint_positions is None:
            rospy.logwarn("Waiting for joint positions...")
            return

        # Check if the new trajectory is the same as the last one
        if self.last_trajectory == self.joint_positions:
            rospy.loginfo("Trajectory unchanged. Waiting for updates...")
            rospy.sleep(2)
            return

        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Add the trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        # Wait 2 seconds and move to home position
        rospy.sleep(2)
        self.move_to_home(trajectory_client)

        self.last_trajectory = self.joint_positions  # Update the last trajectory

    def move_to_home(self, trajectory_client):
        """Move the robot to the home position."""
        rospy.loginfo("Moving to home position.")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES_HOME

        point = JointTrajectoryPoint()
        point.positions = HOME_POSITION
        point.velocities = [0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = JOINT_TRAJECTORY_CONTROLLERS.copy()

        if target_controller in other_controllers:
            other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)

        controller_running = any(
            controller.name == target_controller and controller.state == "running" 
            for controller in response.controller
        )

        if controller_running:
            rospy.loginfo("Controller {} is already running.".format(target_controller))
            return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

if __name__ == "__main__":
    client = TrajectoryClient()
    try:
        while not rospy.is_shutdown():
            # Wait until joint positions are available before sending a trajectory
            client.send_joint_trajectory()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
