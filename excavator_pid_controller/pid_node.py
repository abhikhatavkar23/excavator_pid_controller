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

class PDControllerNode(Node):
    def __init__(self):
        super().__init__('excavator_pd_controller_node')

        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/deltacan')

        self.declare_parameter('boom.kp', 1.5)
        self.declare_parameter('boom.kd', 0.4)
        self.declare_parameter('arm.kp', 1.8)
        self.declare_parameter('arm.kd', 0.5)
        self.declare_parameter('bucket.kp', 1.0)
        self.declare_parameter('bucket.kd', 0.2)

        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('deadband_minimum_power', 0.45)

        self.control_frequency = self.get_parameter('control_frequency').value
        goal_topic = self.get_parameter('goal_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_topic = self.get_parameter('command_topic').value
        
        self.pd_gains = {
            'boom': {'kp': self.get_parameter('boom.kp').value, 'kd': self.get_parameter('boom.kd').value},
            'arm':  {'kp': self.get_parameter('arm.kp').value, 'kd': self.get_parameter('arm.kd').value},
            'bucket': {'kp': self.get_parameter('bucket.kp').value, 'kd': self.get_parameter('bucket.kd').value}
        }
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.deadband_min_power = self.get_parameter('deadband_minimum_power').value

        self.current_positions = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        self.goal_positions = None
        self.last_error = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
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
        
        self.get_logger().info("Excavator Sequential PD Controller has started.")
        self.get_logger().info(f"Listening for goals on: {goal_topic}")

    def goal_callback(self, msg):
        if len(msg.data) == 3:
            self.goal_positions = {'boom': msg.data[0], 'arm': msg.data[1], 'bucket': msg.data[2]}
            self.control_state = ControlState.MOVING_BOOM
            self.get_logger().info(f"New goal received. Starting sequence: MOVING_BOOM")
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

    def calculate_pd_command(self, joint_name):
        error = self.goal_positions[joint_name] - self.current_positions[joint_name]
        final_command = 0.0

        if abs(error) > self.goal_tolerance:
            p_term = self.pd_gains[joint_name]['kp'] * error
            derivative_error = (error - self.last_error[joint_name]) / self.timer_period
            d_term = self.pd_gains[joint_name]['kd'] * derivative_error
            pd_output = p_term + d_term
            
            sign = np.sign(pd_output)
            power_with_deadband = max(self.deadband_min_power, abs(pd_output))
            final_command = np.clip(sign * power_with_deadband, -1.0, 1.0)

        self.last_error[joint_name] = error
        return final_command

    def compute_and_publish_control(self):
        if self.goal_positions is None or self.control_state == ControlState.IDLE:
            return
            
        cmd_msg = DeltaCan()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        
        boom_cmd, arm_cmd, bucket_cmd = 0.0, 0.0, 0.0
        
        if self.control_state == ControlState.MOVING_BOOM:
            boom_cmd = self.calculate_pd_command('boom')
            # If command is zero, it means we've reached the goal
            if np.isclose(boom_cmd, 0.0):
                self.control_state = ControlState.MOVING_ARM
                self.get_logger().info("Boom goal reached. Transitioning to MOVING_ARM.")
        
        elif self.control_state == ControlState.MOVING_ARM:
            arm_cmd = self.calculate_pd_command('arm')
            if np.isclose(arm_cmd, 0.0):
                self.control_state = ControlState.MOVING_BUCKET
                self.get_logger().info("Arm goal reached. Transitioning to MOVING_BUCKET.")

        elif self.control_state == ControlState.MOVING_BUCKET:
            bucket_cmd = self.calculate_pd_command('bucket')
            if np.isclose(bucket_cmd, 0.0):
                self.control_state = ControlState.IDLE
                self.get_logger().info("Bucket goal reached. Sequence complete. Entering IDLE state.")

        cmd_msg.mboomcmd = -1.0 * boom_cmd
        cmd_msg.marmcmd = arm_cmd
        cmd_msg.mbucketcmd = bucket_cmd
        
        self.command_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()