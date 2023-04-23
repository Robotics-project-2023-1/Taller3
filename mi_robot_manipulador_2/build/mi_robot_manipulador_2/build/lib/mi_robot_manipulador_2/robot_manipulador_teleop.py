#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import math
import keyboard

class RobotController(Node):
    def __init__(self, joint_speeds):
        super().__init__('robot_controller')
        self.joint_speeds = joint_speeds
        self.current_joint_positions = [0, 0, 0]
        self.gripper_open = False
        self.zone = ""

        self.joint_positions_pub = self.create_publisher(Vector3, '/joint_positions', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper', 10)

    def update_joint_positions(self, axis_positions):
        # Calculate new joint positions based on joystick or keyboard input
        new_joint_positions = []
        for i in range(len(axis_positions)):
            joint_pos = self.current_joint_positions[i] + self.joint_speeds[i] * axis_positions[i]
            joint_pos = max(-math.pi/2, min(math.pi/2, joint_pos))  # Joint limit range
            new_joint_positions.append(joint_pos)

        # Update current joint positions and publish to topic
        self.current_joint_positions = new_joint_positions
        joint_pos_msg = Vector3(x=new_joint_positions[0], y=new_joint_positions[1], z=new_joint_positions[2])
        self.joint_positions_pub.publish(joint_pos_msg)

    def open_gripper(self):
        # Open gripper with default step size of 2
        gripper_step = 2
        self.gripper_open = True
        gripper_pos = gripper_step
        gripper_msg = String(data=str(gripper_pos))
        self.gripper_pub.publish(gripper_msg)

    def close_gripper(self):
        # Close gripper with default step size of 2
        gripper_step = -2
        self.gripper_open = False
        gripper_pos = gripper_step
        gripper_msg = String(data=str(gripper_pos))
        self.gripper_pub.publish(gripper_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_speeds = [0.5, 0.5, 0.5]  # Default joint speeds
    robot_controller = RobotController(joint_speeds)

    # Register keyboard events to control robot
    keyboard.add_hotkey('up', lambda: robot_controller.update_joint_positions([1, 0, 0]))
    keyboard.add_hotkey('down', lambda: robot_controller.update_joint_positions([-1, 0, 0]))
    keyboard.add_hotkey('left', lambda: robot_controller.update_joint_positions([0, 1, 0]))
    keyboard.add_hotkey('right', lambda: robot_controller.update_joint_positions([0, -1, 0]))
    keyboard.add_hotkey('page up', lambda: robot_controller.update_joint_positions([0, 0, 1]))
    keyboard.add_hotkey('page down', lambda: robot_controller.update_joint_positions([0, 0, -1]))
    keyboard.add_hotkey('space', lambda: robot_controller.open_gripper())
    keyboard.add_hotkey('enter', lambda: robot_controller.close_gripper())

    rclpy.spin(robot_controller)
    robot_controller.destroy
