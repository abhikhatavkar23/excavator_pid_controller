import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from deltacan.msg import DeltaCan
import numpy as np
from enum import Enum

class ControlState(Enum):
    IDLE = 0
    MOVING_BOOM = 1
    MOVING_ARM = 2
    MOVING_BUCKET = 3

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('excavator_pid_controller_node')
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/deltacan')

        # Parameters for proportional control
        self.declare_parameter('large_error_threshold', 0.5)
        self.declare_parameter('medium_error_threshold', 0.25)
        self.declare_parameter('small_error_threshold', 0.05)
        self.declare_parameter('large_error_power', 0.8)
        self.declare_parameter('medium_error_power', 0.6)
        self.declare_parameter('small_error_power', 0.48)

        # get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        goal_topic = self.get_parameter('goal_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_topic = self.get_parameter('command_topic').value
        
        self.control_params = {
            'large_error': self.get_parameter('large_error_threshold').value,
            'medium_error': self.get_parameter('medium_error_threshold').value,
            'small_error': self.get_parameter('small_error_threshold').value,
            'large_power': self.get_parameter('large_error_power').value,
            'medium_power': self.get_parameter('medium_error_power').value,
            'small_error_power': self.get_parameter('small_error_power').value,
        }

        self.current_positions = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        self.goal_positions = None
        self.control_state = ControlState.IDLE
        
        self.joint_indices = {}
        self.joint_names_to_find = ['swing_to_boom', 'boom_to_arm', 'arm_to_bucket']

        self.goal_subscriber = self.create_subscription(
            Float64MultiArray, goal_topic, self.goal_callback, 10)
        
        self.joint_state_subscriber = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10)
        
        self.command_publisher = self.create_publisher(
            DeltaCan, command_topic, 10)
            
        self.timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(self.timer_period, self.compute_and_publish_control)
        
        self.get_logger().info("✅ Excavator Sequential Controller has started.")
        self.get_logger().info(f"Listening for goals on: {goal_topic}")

    def goal_callback(self, msg):
        if len(msg.data) == 3:
            self.goal_positions = {'boom': msg.data[0], 'arm': msg.data[1], 'bucket': msg.data[2]}
            self.control_state = ControlState.MOVING_BOOM
            self.get_logger().info(f"🎯 New goal received. Starting sequence: MOVING_BOOM")
        else:
            self.get_logger().warning(f"Received goal with {len(msg.data)} elements, expected 3.")

    def joint_state_callback(self, msg: JointState):
        if not self.joint_indices:
            try:
                self.joint_indices['boom'] = msg.name.index(self.joint_names_to_find[0])
                self.joint_indices['arm'] = msg.name.index(self.joint_names_to_find[1])
                self.joint_indices['bucket'] = msg.name.index(self.joint_names_to_find[2])
                self.get_logger().info(f"Successfully mapped joint indices: {self.joint_indices}")
            except ValueError as e:
                self.get_logger().error(f"Joint name not found in /joint_states message: {e}")
                return

        self.current_positions['boom'] = msg.position[self.joint_indices['boom']]
        self.current_positions['arm'] = msg.position[self.joint_indices['arm']]
        self.current_positions['bucket'] = msg.position[self.joint_indices['bucket']]

        if self.goal_positions is None:
            self.goal_positions = self.current_positions.copy()
            self.get_logger().info(f"Initialized goal to current positions.")

    def compute_and_publish_control(self):
        if self.goal_positions is None or self.control_state == ControlState.IDLE:
            return
            
        cmd_msg = DeltaCan()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        
        boom_cmd, arm_cmd, bucket_cmd = 0.0, 0.0, 0.0
        
        if self.control_state == ControlState.MOVING_BOOM:
            error = self.goal_positions['boom'] - self.current_positions['boom']
            boom_cmd = self.calculate_power(error)
            if abs(error) < self.control_params['small_error']:
                self.control_state = ControlState.MOVING_ARM
                self.get_logger().info("Boom goal reached. Transitioning to MOVING_ARM.")
        
        elif self.control_state == ControlState.MOVING_ARM:
            error = self.goal_positions['arm'] - self.current_positions['arm']
            arm_cmd = self.calculate_power(error)
            if abs(error) < self.control_params['small_error']:
                self.control_state = ControlState.MOVING_BUCKET
                self.get_logger().info("Arm goal reached. Transitioning to MOVING_BUCKET.")

        elif self.control_state == ControlState.MOVING_BUCKET:
            error = self.goal_positions['bucket'] - self.current_positions['bucket']
            bucket_cmd = self.calculate_power(error)
            if abs(error) < self.control_params['small_error']:
                self.control_state = ControlState.IDLE
                self.get_logger().info("Bucket goal reached. Sequence complete. Entering IDLE state.")

        cmd_msg.mboomcmd = -1.0 * boom_cmd
        cmd_msg.marmcmd = arm_cmd
        cmd_msg.mbucketcmd = bucket_cmd
        
        self.command_publisher.publish(cmd_msg)

    def calculate_power(self, error):
        abs_error = abs(error)
        power = 0.0
        if abs_error > self.control_params['large_error']:
            power = self.control_params['large_power']
        elif abs_error > self.control_params['medium_error']:
            power = self.control_params['medium_power']
        elif abs_error > self.control_params['small_error']:
            power = self.control_params['small_error_power']
        
        return power * np.sign(error)

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