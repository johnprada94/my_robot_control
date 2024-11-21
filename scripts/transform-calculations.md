# UR10 Robot Transformation and IK Calculation

This repository contains code to compute the pose of an object relative to the UR10 robot's base frame. The transformation matrices provided are used to calculate the object's pose for pick-and-place tasks, ensuring the end effector faces downward.

---

## **Mathematics Behind the Code**

### **Transformation Matrices**
In robotics, a **transformation matrix** represents the position and orientation of one frame relative to another. A 4x4 transformation matrix \( T \) consists of:
- **3x3 rotation matrix \( R \)**: Represents orientation.
- **3x1 translation vector \( t \)**: Represents position.

\[
T = 
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
\]

### **Combining Transformations**
To compute the pose of an object in the robot's base frame, the following relationship is used:

\[T_{\text{base} \to \text{object}} = T_{\text{base} \to \text{ee}} \cdot T_{\text{ee} \to \text{camera}} \cdot T_{\text{camera} \to \text{object}}\]

Where:
- \( T_{\text{base} \to \text{ee}} \): Pose of the end effector relative to the robot's base frame.
- \( T_{\text{ee} \to \text{camera}} \): Inverse of \( T_{\text{camera} \to \text{ee}} \), transforming from the camera frame to the end effector frame.
- \( T_{\text{camera} \to \text{object}} \): Pose of the object relative to the camera.

### **Object Orientation Adjustment**
The Z-axis of the calculated object pose is modified to ensure the end effector aligns downward during the pick-up operation. This ensures proper orientation for grasping.

---

## **How to Run**

### **1. Launch the UR10 Robot Simulation**
In a new terminal, run the following command:
```bash
roslaunch ur10_new demo.launch use_robot_state_publisher:=false
