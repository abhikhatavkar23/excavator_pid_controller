# Excavator PID Controller

This ROS 2 package provides a simple PID (Proportional-Integral-Derivative) controller to perform position control for an excavator's boom, arm, and bucket joints. The controller subscribes to the current joint states and a target goal pose, and it publishes velocity commands to drive the joints to the desired position.

![Sim Tested](https://img.shields.io/badge/sim--tested-blue?style=flat-square)
![Status](https://img.shields.io/badge/status-stable-brightgreen?style=flat-square)

---

## 🚧 Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- A working excavator simulator that publishes `control_msgs/msg/DynamicJointState` and accepts `std_msgs/msg/Float64MultiArray` for velocity commands.

---

## 🛠️ Installation & Building

1.  **Clone the Repository**
    Navigate to your ROS 2 workspace's `src` directory and clone the repository.
    ```bash
    cd ~/your_ros2_ws/src/
    git clone [https://github.com/abhikhatavkar23/excavator_pid_controller.git](https://github.com/abhikhatavkar23/excavator_pid_controller.git)
    ```

2.  **Build the Package**
    Navigate to the root of your workspace and build the package using `colcon`.
    ```bash
    cd ~/your_ros2_ws/
    colcon build --packages-select excavator_pid_controller
    ```

3.  **Source the Workspace**
    Before running the node, source your workspace's setup file.
    ```bash
    source ~/your_ros2_ws/install/setup.bash
    ```

---

## 🚀 Usage

1.  **Launch the Simulator**
    In a new terminal, start your excavator simulator.

2.  **Launch the PID Controller**
    In a second terminal (after sourcing the workspace), run the controller's launch file. This will start the node and load the PID gains from the config file.
    ```bash
    ros2 launch excavator_pid_controller pid_controller.launch.py
    ```
    You should see the message `Excavator PID Controller has started.`

3.  **Send a Goal Command**
    In a third terminal (after sourcing), you can command the excavator to move to a new position by publishing to the `/goal_pose` topic. The message data is an array of three numbers representing the target angles in **radians** for `[boom, arm, bucket]`.

    **Example:** Move the boom to -0.2 rad, the arm to -0.5 rad, and the bucket to 0.6 rad.
    ```bash
    ros2 topic pub --once /goal_pose std_msgs/msg/Float64MultiArray "{data: [-0.2, -0.5, 0.6]}"
    ```
    The excavator will now move to the commanded position.

---

## ⚙️ Node Architecture

### `excavator_pid_controller_node`

#### Subscribed Topics
-   **`/dynamic_joint_states`** (`control_msgs/msg/DynamicJointState`)
    * Listens for the current position of all excavator joints to calculate the error.
-   **`/goal_pose`** (`std_msgs/msg/Float64MultiArray`)
    * Receives the target joint angles `[boom, arm, bucket]` in radians.

#### Published Topics
-   **`/manipulator_controller/commands`** (`std_msgs/msg/Float64MultiArray`)
    * Publishes the calculated velocity commands `[boom_vel, arm_vel, bucket_vel]` to the simulator's controller.

---

## 🔧 Configuration & Tuning

All parameters for the controller can be modified in the `config/pid_params.yaml` file.

### PID Gains
The core of the controller's behavior is defined by the PID gains for each joint. You can tune these values to change how the excavator moves.

-   `kp` (Proportional Gain): The primary driving force. Higher values result in faster movement but can cause overshoot.
-   `ki` (Integral Gain): Eliminates small, steady-state errors. Use small values to prevent instability.
-   `kd` (Derivative Gain): Acts as a damper to reduce oscillations and prevent overshoot.

To apply changes to the gains, you must edit the `pid_params.yaml` file and then rebuild the package with `colcon build`.
