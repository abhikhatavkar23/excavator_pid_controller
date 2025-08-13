import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('excavator_pid_controller_node')

        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('joint_states_topic', '/dynamic_joint_states')
        self.declare_parameter('command_topic', '/manipulator_controller/commands') # Command topic for gazebo controller
        self.declare_parameter('joint_names', ['swing_to_boom', 'boom_to_arm', 'arm_to_bucket'])
        
        # PID gains for each joint
        self.declare_parameter('boom.kp', 5.0)
        self.declare_parameter('boom.ki', 0.1)
        self.declare_parameter('boom.kd', 0.5)
        self.declare_parameter('arm.kp', 5.0)
        self.declare_parameter('arm.ki', 0.1)
        self.declare_parameter('arm.kd', 0.5)
        self.declare_parameter('bucket.kp', 3.0)
        self.declare_parameter('bucket.ki', 0.05)
        self.declare_parameter('bucket.kd', 0.2)

        self.control_frequency = self.get_parameter('control_frequency').value
        goal_topic = self.get_parameter('goal_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_topic = self.get_parameter('command_topic').value
        self.joint_names = self.get_parameter('joint_names').value
        
        # store PID
        self.pid_gains = {
            'boom': {'kp': self.get_parameter('boom.kp').value, 'ki': self.get_parameter('boom.ki').value, 'kd': self.get_parameter('boom.kd').value},
            'arm':  {'kp': self.get_parameter('arm.kp').value, 'ki': self.get_parameter('arm.ki').value, 'kd': self.get_parameter('arm.kd').value},
            'bucket': {'kp': self.get_parameter('bucket.kp').value, 'ki': self.get_parameter('bucket.ki').value, 'kd': self.get_parameter('bucket.kd').value}
        }
        
        #State variables
        self.current_positions = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        self.goal_positions = None # No goal initially
        
        # PID internal state
        self.integral_error = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        self.last_error = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        
        # Store the indices of the joints
        self.joint_indices = {}

        #ROS2 communications
        self.goal_subscriber = self.create_subscription(
            Float64MultiArray, goal_topic, self.goal_callback, 10)
        
        self.joint_state_subscriber = self.create_subscription(
            DynamicJointState, joint_states_topic, self.joint_state_callback, 10)
        
        self.command_publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10)
            
        # control loop
        self.timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(self.timer_period, self.compute_and_publish_control)
        
        self.get_logger().info("Excavator PID Controller has started.")
        self.get_logger().info(f"Listening for goals on: {goal_topic}")
        self.get_logger().info(f"Publishing commands to: {command_topic}")

    def goal_callback(self, msg):
        """Receives a new goal pose (boom, arm, bucket angles)."""
        if len(msg.data) == 3:
            self.goal_positions = {'boom': msg.data[0], 'arm': msg.data[1], 'bucket': msg.data[2]}
            #reset PID integrals when a new goal is set to prevent wind-up
            self.integral_error = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
            self.get_logger().info(f"New goal received: Boom={msg.data[0]:.2f}, Arm={msg.data[1]:.2f}, Bucket={msg.data[2]:.2f}")
        else:
            self.get_logger().warning(f"Received goal with {len(msg.data)} elements, expected 3.")

    def joint_state_callback(self, msg: DynamicJointState):
        """Updates the current positions of the excavator joints."""

        #if haven't found joint indices, find them
        if not self.joint_indices:
            try:
                self.joint_indices['boom'] = msg.joint_names.index(self.joint_names[0])
                self.joint_indices['arm'] = msg.joint_names.index(self.joint_names[1])
                self.joint_indices['bucket'] = msg.joint_names.index(self.joint_names[2])
                self.get_logger().info("Found all target joint indices.")
            except ValueError as e:
                self.get_logger().error(f"Joint name from parameters not found in /dynamic_joint_states: {e}")
                return

        # Extract position for each joint
        for joint_name, index in self.joint_indices.items():
            # The position is the first value in the 'values' list for that joint
            self.current_positions[joint_name] = msg.interface_values[index].values[0]

        # If this is the first time we get joint states and have no goal, set goal to current state
        if self.goal_positions is None:
            self.goal_positions = self.current_positions.copy()
            self.get_logger().info(f"Initialized goal to current positions.")

    def compute_and_publish_control(self):
        """The main PID control loop."""
        if self.goal_positions is None or not self.joint_indices:
            # Don't do anything until we have a goal and know the joint states
            return
            
        control_commands = []
        joint_order = ['boom', 'arm', 'bucket'] # Ensure consistent order

        for joint in joint_order:
            # --- PID Calculation ---
            error = self.goal_positions[joint] - self.current_positions[joint]
            
            # Proportional term
            p_term = self.pid_gains[joint]['kp'] * error
            
            # Integral term (with anti-windup via clamping)
            self.integral_error[joint] += error * self.timer_period
            # You can add clamping here if needed, e.g., self.integral_error[joint] = max(min(self.integral_error[joint], 5.0), -5.0)
            i_term = self.pid_gains[joint]['ki'] * self.integral_error[joint]
            
            # Derivative term
            derivative_error = (error - self.last_error[joint]) / self.timer_period
            d_term = self.pid_gains[joint]['kd'] * derivative_error
            
            # Update last error
            self.last_error[joint] = error
            
            # Combine terms to get command assuming the '/manipulator_controller/commands' topic takes velocity commands.
            command = p_term + i_term + d_term
            
            # Clamp the output to reasonable limits, e.g. [-2.0, 2.0] rad/s
            command = np.clip(command, -2.0, 2.0)
            
            control_commands.append(command)

        # publish the commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = control_commands
        self.command_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()