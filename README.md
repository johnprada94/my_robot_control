# my_robot_control

This package is designed for controlling the UR10 robot and establishing bidirectional communication with Delfoi-ROS.

## Setup Instructions

### For Robot Calibration and Control

1. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the calibration correction launch file:
     ```bash
     roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.0.100 target_filename:="${HOME}/my_robot_calibration.yaml"
     ```

   2. **In a new terminal:**

      - Source the workspace:
        ```bash
        source ~/catkin_ws/devel/setup.bash
        ```

      - Run the UR10 bringup launch file:
        ```bash
        roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=REVERSE_PORT] kinematics_config:=${HOME}/my_robot_calibration.yaml
        ```

   3. **Activate external control** in the teach pendant of the UR10.

### For Bidirectional Communication with Delfoi-ROS

1. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the command to echo joint states:
     ```bash
     rostopic echo /joint_states
     ```

2. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the UR10 joint publisher:
     ```bash
     rosrun my_robot_control ur10-joint-publisher.py
     ```

3. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the TCP server:
     ```bash
     rosrun my_robot_control tcp-server.py
     ```

### For Sending Joint Positions to Delfoi

1. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the loop trajectory command:
     ```bash
     rosrun my_robot_control loop_trajectory.py
     ```

### For Receiving Positions from Delfoi

1. **In a new terminal:**

   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

   - Run the move-robot script:
     ```bash
     rosrun my_robot_control move-robot-delfoi-master.py
     ```

## License
This project is licensed under the MIT License.
