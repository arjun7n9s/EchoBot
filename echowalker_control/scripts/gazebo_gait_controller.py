#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GazeboGaitController(Node):
    '''
    Controls the robot's gait in Gazebo using effort controllers.
    Implements a trotting gait pattern for the quadruped robot.
    '''
    
    def __init__(self):
        super().__init__('gazebo_gait_controller')
        
        # Declare parameters
        self.declare_parameter(
            'walking_speed', 
            0.3,  # Default value (0.0 to 1.0)
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Walking speed factor (0.0 to 1.0)'
            )
        )
        
        self.declare_parameter(
            'direction_topic', 
            '/sound_direction',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic to subscribe for sound direction'
            )
        )
        
        self.declare_parameter(
            'gait_cycle_duration', 
            2.0,  # seconds
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Duration of one complete gait cycle'
            )
        )
        
        self.declare_parameter(
            'step_height', 
            0.05,  # meters
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum height of foot during swing phase'
            )
        )
        
        self.declare_parameter(
            'effort_scaling', 
            20.0,  # Effort multiplier for joint commands
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Scaling factor for effort commands'
            )
        )
        
        # Get parameter values
        self.walking_speed = self.get_parameter('walking_speed').value
        self.direction_topic = self.get_parameter('direction_topic').value
        self.gait_cycle_duration = self.get_parameter('gait_cycle_duration').value
        self.step_height = self.get_parameter('step_height').value
        self.effort_scaling = self.get_parameter('effort_scaling').value
        
        # Set up QoS profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher for joint effort commands
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            qos
        )
        
        # Subscribe to joint states for feedback
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        
        # Subscribe to sound direction
        self.direction_subscription = self.create_subscription(
            PoseStamped,
            self.direction_topic,
            self.direction_callback,
            10
        )
        
        # Set up timer for gait control
        self.control_timer = self.create_timer(
            0.02,  # 50Hz control loop
            self.control_callback
        )
        
        # Initialize robot state
        self.current_time = 0.0
        self.phase_offset = {
            'front_right': 0.0,
            'front_left': 0.5,   # 180 degrees out of phase with front_right
            'rear_left': 0.0,    # In phase with front_right
            'rear_right': 0.5    # In phase with front_left
        }
        
        # Default walking direction (forward)
        self.direction = np.array([1.0, 0.0, 0.0])
        self.turning_angle = 0.0
        
        # Robot dimensions (from URDF)
        self.leg_offset_x = 0.15  # Distance from center to hip joint in x
        self.leg_offset_y = 0.13  # Distance from center to hip joint in y
        self.upper_leg_length = 0.15
        self.lower_leg_length = 0.15
        
        # Standing height when legs are straight down
        self.standing_height = self.upper_leg_length + self.lower_leg_length
        
        # Default standing pose with slight bend in knees (more stable)
        self.standing_knee_angle = -0.2  # radians
        self.standing_hip_angle = 0.0
        
        # Neutral foot positions relative to hip joints
        self.neutral_foot_positions = {
            'front_right': np.array([0.05, -0.05, -self.standing_height * 0.85]),
            'front_left': np.array([0.05, 0.05, -self.standing_height * 0.85]),
            'rear_right': np.array([-0.05, -0.05, -self.standing_height * 0.85]),
            'rear_left': np.array([-0.05, 0.05, -self.standing_height * 0.85])
        }
        
        # Step length and width (adjusted by speed)
        self.step_length = 0.1  # meters (will be scaled by speed)
        self.step_width = 0.02  # meters (side-to-side movement)
        
        # Current joint positions for PD control
        self.current_joint_positions = {}
        self.target_joint_positions = {}
        
        # Joint order for effort controller
        self.joint_names = [
            'front_right_hip_joint',
            'front_right_knee_joint',
            'front_left_hip_joint',
            'front_left_knee_joint',
            'rear_right_hip_joint',
            'rear_right_knee_joint',
            'rear_left_hip_joint',
            'rear_left_knee_joint'
        ]
        
        # PD gains for effort control
        self.kp = 100.0  # Proportional gain
        self.kd = 10.0   # Derivative gain
        
        # Initialize joint tracking
        for joint_name in self.joint_names:
            self.current_joint_positions[joint_name] = 0.0
            self.target_joint_positions[joint_name] = 0.0
        
        self.get_logger().info("Gazebo gait controller initialized")
    
    def joint_state_callback(self, msg):
        '''Callback for joint state feedback'''
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.current_joint_positions:
                self.current_joint_positions[joint_name] = msg.position[i]
    
    def direction_callback(self, msg):
        '''Callback for sound direction messages'''
        # Extract direction vector from the message
        self.direction = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Normalize direction
        direction_norm = np.linalg.norm(self.direction)
        if direction_norm > 0:
            self.direction = self.direction / direction_norm
        
        # Calculate turning angle (yaw) from direction
        self.turning_angle = math.atan2(self.direction[1], self.direction[0])
        
        self.get_logger().debug(f"Received direction: {self.direction}, turning angle: {self.turning_angle}")
    
    def control_callback(self):
        '''Main control loop for generating gait pattern'''
        # Update time
        self.current_time += 0.02  # 50Hz control
        if self.current_time > self.gait_cycle_duration:
            self.current_time -= self.gait_cycle_duration
        
        # Normalize time to the gait cycle (0.0 to 1.0)
        cycle_phase = self.current_time / self.gait_cycle_duration
        
        # Apply walking speed to step length
        effective_step_length = self.step_length * self.walking_speed
        
        # Calculate target joint positions for each leg
        leg_order = ['front_right', 'front_left', 'rear_right', 'rear_left']
        
        for leg_name in leg_order:
            # Calculate leg phase (0.0 to 1.0) considering the offsets
            leg_phase = (cycle_phase + self.phase_offset[leg_name]) % 1.0
            
            # Get foot trajectory point for this phase
            foot_position = self.calculate_foot_position(leg_name, leg_phase, effective_step_length)
            
            # Convert foot position to joint angles using inverse kinematics
            hip_angle, knee_angle = self.inverse_kinematics(foot_position)
            
            # Update target positions
            self.target_joint_positions[f"{leg_name}_hip_joint"] = hip_angle
            self.target_joint_positions[f"{leg_name}_knee_joint"] = knee_angle
        
        # Calculate and publish effort commands
        self.publish_effort_commands()
    
    def calculate_foot_position(self, leg_name, phase, step_length):
        '''
        Calculate the foot position for a given leg at a specific phase.
        Implements a typical quadruped gait pattern with stance and swing phases.
        '''
        # Get neutral foot position for this leg
        neutral_position = self.neutral_foot_positions[leg_name].copy()
        
        # Determine if we're in swing phase (0.0-0.5) or stance phase (0.5-1.0)
        is_swing_phase = phase < 0.5
        
        if is_swing_phase:
            # Swing phase - foot moves forward and up in an arc
            swing_phase = phase * 2.0  # Normalize to 0.0-1.0 within swing phase
            
            # X movement (forward/backward)
            x_offset = step_length * (swing_phase - 0.5)
            
            # Z movement (up/down in sine wave pattern)
            z_offset = self.step_height * math.sin(swing_phase * math.pi)
            
            # Y movement (small side-to-side motion for stability)
            y_offset = self.step_width * math.sin(swing_phase * math.pi)
            
        else:
            # Stance phase - foot moves backward in contact with ground
            stance_phase = (phase - 0.5) * 2.0  # Normalize to 0.0-1.0 within stance phase
            
            # X movement (backward during stance, linear)
            x_offset = step_length * (0.5 - stance_phase)
            
            # No Z movement during stance (foot on ground)
            z_offset = 0.0
            
            # Minimal Y movement during stance
            y_offset = 0.0
        
        # Apply turning effect by rotating the foot position
        if abs(self.turning_angle) > 0.05:  # Only apply turning if angle is significant
            # Front legs move more when turning
            turn_factor = 1.5 if 'front' in leg_name else 1.0
            
            # Left legs and right legs move in opposite directions when turning
            side_factor = 1.0 if 'left' in leg_name else -1.0
            
            # Apply turning offset to X and Y
            turn_offset_x = math.sin(self.turning_angle) * 0.03 * turn_factor * side_factor
            turn_offset_y = (1.0 - math.cos(self.turning_angle)) * 0.03 * turn_factor * side_factor
            
            x_offset += turn_offset_x
            y_offset += turn_offset_y
        
        # Apply the movement to neutral position
        foot_position = neutral_position.copy()
        foot_position[0] += x_offset
        foot_position[1] += y_offset
        foot_position[2] += z_offset
        
        return foot_position
    
    def inverse_kinematics(self, foot_position):
        '''
        Calculate inverse kinematics for a leg.
        
        Args:
            foot_position: Target foot position in leg's local coordinates
            
        Returns:
            hip_angle, knee_angle in radians
        '''
        x, y, z = foot_position
        
        # Calculate the distance from hip to foot in the XZ plane
        l = math.sqrt(x*x + z*z)
        
        # Calculate the hip angle in the XZ plane
        hip_angle_xz = math.atan2(x, -z)
        
        # Use the law of cosines to calculate the knee angle
        d = math.sqrt(l*l + y*y)  # Direct distance from hip to foot
        
        # Check if the target is reachable
        if d > (self.upper_leg_length + self.lower_leg_length):
            # Target too far, stretch the leg as much as possible
            knee_angle = 0.0
        else:
            # Calculate knee angle using law of cosines
            cos_knee = (self.upper_leg_length**2 + self.lower_leg_length**2 - d**2) / (2 * self.upper_leg_length * self.lower_leg_length)
            
            # Clamp to valid range due to floating point errors
            cos_knee = max(-1.0, min(1.0, cos_knee))
            
            # Knee bends backward (negative angle)
            knee_angle = -(math.pi - math.acos(cos_knee))
        
        # Calculate the hip angle in the YZ plane
        hip_angle_yz = math.atan2(y, math.sqrt(x*x + z*z))
        
        # Combine the hip angles
        hip_angle = hip_angle_xz
        
        return hip_angle, knee_angle
    
    def publish_effort_commands(self):
        '''Calculate and publish effort commands using PD control'''
        effort_msg = Float64MultiArray()
        efforts = []
        
        for joint_name in self.joint_names:
            # Get current and target positions
            current_pos = self.current_joint_positions.get(joint_name, 0.0)
            target_pos = self.target_joint_positions.get(joint_name, 0.0)
            
            # Calculate position error
            pos_error = target_pos - current_pos
            
            # Simple PD control (derivative term approximated as zero for now)
            effort = self.kp * pos_error
            
            # Scale effort
            effort *= self.effort_scaling
            
            # Clamp effort to reasonable limits
            effort = max(-50.0, min(50.0, effort))
            
            efforts.append(effort)
        
        effort_msg.data = efforts
        self.effort_publisher.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGaitController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()