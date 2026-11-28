"""Publish an RViz-only G1 joint state on a regular ROS timer."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .kinematics import DrummerKinematics, Hit


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('drummer')
        default_chart = (
            Path(get_package_share_directory('unitree_drummer'))
            / 'config' / 'test_groove.json'
        )
        description = self.declare_parameter('robot_description', '').value
        chart_path = self.declare_parameter('chart', str(default_chart)).value
        chart = json.loads(Path(chart_path).read_text(encoding='utf-8'))
        self.hits = tuple(sorted(
            (Hit(**event) for event in chart['hits']), key=lambda hit: hit.time_s,
        ))

        # Define joint names in the same order as the published joint angles.
        robot = ET.fromstring(description)
        self.jointnames = [
            joint.attrib['name'] for joint in robot.findall('joint')
            if joint.attrib['type'] != 'fixed'
        ]

        # Set up the kinematics for the waist and arms.
        self.active_indices = tuple(
            i for i, name in enumerate(self.jointnames)
            if name.startswith('waist_')
            or any(part in name for part in ('_shoulder_', '_elbow_', '_wrist_'))
        )
        self.active_joints = tuple(self.jointnames[i] for i in self.active_indices)
        self.kinematics = DrummerKinematics(description, self.active_joints)

        # Define the initial seated pose and copy it into the joint command.
        self.q0 = np.zeros(len(self.jointnames))
        for side in ('left', 'right'):
            self.q0[self.jointnames.index(f'{side}_hip_pitch_joint')] = -1.57
            self.q0[self.jointnames.index(f'{side}_knee_joint')] = 1.57
        self.qc = self.q0.copy()

        # Set up the joint-state publisher and the 100 Hz timer.
        self.pubjoint = self.create_publisher(JointState, 'joint_states', 10)
        self.dt = 0.01
        self.start_time = None
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info(
            f'{chart["title"]}: {len(self.hits)} target hits. '
            'The starter kinematics holds the seated pose.'
        )

    def update(self):
        now = self.get_clock().now()
        if self.start_time is None and self.pubjoint.get_subscription_count():
            self.start_time = now
        if self.start_time is not None:
            time_s = (now - self.start_time).nanoseconds / 1e9 - 2.0
            previous = {
                self.jointnames[i]: float(self.qc[i]) for i in self.active_indices
            }
            command = self.kinematics.update(time_s, self.hits, previous)
            for i in self.active_indices:
                self.qc[i] = command[self.jointnames[i]]

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = self.jointnames
        message.position = self.qc.tolist()
        self.pubjoint.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
