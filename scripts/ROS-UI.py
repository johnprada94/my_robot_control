import subprocess
import tkinter as tk

# Function to run a ROS script in a new Terminator terminal with a custom tab name
def run_ros_script(script_name):
    # Adjust the path to your ROS setup files
    source_cmd = "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash"
    ros_cmd = f"rosrun my_robot_control {script_name}"

    # Run Terminator with a custom title and execute the ROS script
    subprocess.Popen([
        "terminator",
        "--new-tab", 
        "--title", f"Running {script_name}",  # Set the tab title
        "-e", f"bash -c '{source_cmd} && {ros_cmd}; exec bash'"  # Execute the ROS script
    ])

# Function to run `rostopic echo` in a new tab with a custom name
def split_and_run_rostopic():
    # Command to run rostopic echo in a new tab
    source_cmd = "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash"
    rostopic_cmd = f"rostopic echo /joint_states"

    # Run Terminator with a custom title and execute rostopic echo
    subprocess.Popen([
        "terminator",
        "--new-tab", 
        "--title", "Joint States Echo",  # Custom title for the tab
        "-e", f"bash -c '{source_cmd} && {rostopic_cmd}; exec bash'"  # Execute the command in the new tab
    ])

# Function to launch a ROS launch file in a new Terminator tab
def launch_ros_file():
    # Command to launch the roslaunch file with the required parameters
    source_cmd = "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash"
    launch_cmd = "roslaunch ur10_new demo.launch use_robot_state_publisher:=false"

    # Run Terminator with a custom title and execute the launch command
    subprocess.Popen([
        "terminator",
        "--new-tab", 
        "--title", "UR10 Launch",  # Custom title for the tab
        "-e", f"bash -c '{source_cmd} && {launch_cmd}; exec bash'"  # Execute the roslaunch command
    ])


# Create the main window for the UI
root = tk.Tk()
root.title("UR10 Control")

# Button 1: Echo Joint States
button1 = tk.Button(root, text="Echo Joint States", command=split_and_run_rostopic)
button1.pack(pady=10)

# Button 2: Echo Joint States degree
button2 = tk.Button(root, text="Echo Joint States (degree)", command=lambda: run_ros_script("ur10-joint-publisher.py"))
button2.pack(pady=10)

# Button 3: Start Server
button3 = tk.Button(root, text="Start Server", command=lambda: run_ros_script("tcp-server.py"))
button3.pack(pady=10)

# Button 4: Start MoveIt
button4 = tk.Button(root, text="Launch UR10 Moveit", command=launch_ros_file)
button4.pack(pady=10)

# Button 5: Start transform calculation from the camera
button5 = tk.Button(root, text="Transform Calculation", command=lambda: run_ros_script("transform-full.py"))
button5.pack(pady=10)

# Button 6: Start IK calculation from the camera
button6 = tk.Button(root, text="IK Calculation", command=lambda: run_ros_script("ur10-ik.py"))
button6.pack(pady=10)

# Button 7: Pick-and-Place
button7 = tk.Button(root, text="Pick object", command=lambda: run_ros_script("ur10-action.py"))
button7.pack(pady=10)

# Run the UI
root.mainloop()
