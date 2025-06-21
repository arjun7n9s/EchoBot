#!/usr/bin/env python3

import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class SoundSimulator(Node):
    """
    Simulates a sound source and publishes its direction.
    The sound source moves in a circular pattern around the robot.
    Compatible with ROS2 Jazzy.
    """

    def __init__(self):
        super().__init__('sound_simulator')

        # Declare parameters
        self.declare_parameter(
            'publish_rate', 
            10.0,  # Default value in Hz
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Rate at which to publish sound direction updates'
            )
        )

        self.declare_parameter(
            'topic_name', 
            '/sound_direction',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic name for publishing sound direction'
            )
        )

        self.declare_parameter(
            'circle_radius', 
            3.0,  # meters
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Radius of the circle pattern for sound source movement'
            )
        )

        self.declare_parameter(
            'circle_period', 
            30.0,  # seconds
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Time for sound source to complete one circle'
            )
        )

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('topic_name').value
        self.circle_radius = self.get_parameter('circle_radius').value
        self.circle_period = self.get_parameter('circle_period').value

        # Create publishers
        self.direction_publisher = self.create_publisher(
            PoseStamped, 
            self.topic_name, 
            10
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            '/sound_direction_marker',
            10
        )

        # Set up timer for publishing
        self.timer = self.create_timer(
            1.0/self.publish_rate,
            self.timer_callback
        )

        # Initialize sound source position
        self.angle = 0.0
        self.get_logger().info(f"Sound simulator started, publishing to {self.topic_name} at {self.publish_rate} Hz")

    def timer_callback(self):
        """Timer callback to publish sound direction and visualization marker"""
        # Update sound source position (circular motion)
        self.angle += 2.0 * math.pi / (self.circle_period * self.publish_rate)
        if self.angle > 2.0 * math.pi:
            self.angle -= 2.0 * math.pi

        # Calculate sound source position
        x = self.circle_radius * math.cos(self.angle)
        y = self.circle_radius * math.sin(self.angle)
        z = 0.0  # Same height as robot

        # Create direction message (pose oriented toward sound)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sound_sensor'

        # Calculate direction vector
        direction = np.array([x, y, z])
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 0:
            direction = direction / direction_norm

        # Convert direction to quaternion (using a simplified approach here)
        # This assumes the sound sensor is on the front of the robot pointing forward
        yaw = math.atan2(direction[1], direction[0])

        # For simplicity, we're just using the yaw to represent the direction
        # In a real implementation, you'd convert to a proper quaternion
        msg.pose.position.x = direction[0]
        msg.pose.position.y = direction[1]
        msg.pose.position.z = direction[2]

        # Publish the direction
        self.direction_publisher.publish(msg)

        # Publish visualization marker
        self.publish_marker(x, y, z)

    def publish_marker(self, x, y, z):
        """Publish a visualization marker for RViz"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sound_source"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position of the sound source
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)

        # Color (red)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(1e9 / self.publish_rate)

        # Publish the marker
        self.marker_publisher.publish(marker)

        # Also publish a line from robot to sound source
        line_marker = Marker()
        line_marker.header.frame_id = "sound_sensor"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "sound_direction"
        line_marker.id = 1
        line_marker.type = Marker.ARROW
        line_marker.action = Marker.ADD

        # Start at sound sensor
        line_marker.points.append(Point(x=0.0, y=0.0, z=0.0))

        # Direction vector to sound source, normalized to a fixed length for visualization
        length = 1.0  # 1 meter arrow
        direction = np.array([x, y, z])
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 0:
            direction = direction * length / direction_norm

        # End at the normalized direction
        line_marker.points.append(Point(
            x=direction[0],
            y=direction[1],
            z=direction[2]
        ))

        # Scale (width of arrow)
        line_marker.scale = Vector3(x=0.05, y=0.1, z=0.1)

        # Color (yellow)
        line_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)

        # Lifetime
        line_marker.lifetime.sec = 0
        line_marker.lifetime.nanosec = int(1e9 / self.publish_rate)

        # Publish the line marker
        self.marker_publisher.publish(line_marker)


def main(args=None):
    rclpy.init(args=args)
    node = SoundSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
