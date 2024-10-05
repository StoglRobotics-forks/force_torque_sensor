#!/usr/bin/env python

from dataclasses import dataclass
from enum import Enum
from math import sqrt
import time
import threading
from typing import List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3, WrenchStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler
from transforms3d.quaternions import quat2mat


import numpy as np


KUKA = "kuka"
UR = "ur"
YASKAWA = "yaskawa"


def positions_diff(a: List[float], b: List[float]) -> List[float]:
    return [abs(a[i] - b[i]) for i in range(len(a))]


def positions_near(a: List[float], b: List[float], eps=0.04) -> bool:
    return all([diff < eps for diff in positions_diff(a, b)])


class CalibrationNode(Node):

    def __init__(self):
        super().__init__("calibrate_tool_node")


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        self.CoG_frame = ""
        self.CoG = Vector3()


        ### PARAMS:
        self.robot = self.declare_parameter("robot", "yaskawa").value
        self.joint_names = self.declare_parameter(
            "joint_names",
            [
                "joint_1_s",
                "joint_2_l",
                "joint_3_u",
                "joint_4_r",
                "joint_5_b",
                "joint_6_t",
            ],
        ).value
        self.controller_topic = self.declare_parameter(
            "controller_topic", "/position_trajectory_controller/joint_trajectory"
        ).value
        self.wrench_topic = self.declare_parameter(
            "wrench_topic", "/force_torque_sensor_broadcaster/raw_wrench"
        ).value
        self.joint_states_topic = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value
        self.max_allowed_velocity_rad_per_sec = self.declare_parameter(
            "max_allowed_velocity_rad_per_sec", 0.1
        ).value

        self.data_mutex = threading.Lock()

        self.trajectory_pub_ = self.create_publisher(
            JointTrajectory, self.controller_topic, 10
        )

        self.wrench_msg = WrenchStamped()
        self.wrench_sub_ = self.create_subscription(
            WrenchStamped, self.wrench_topic, self.wrench_callback, 10
        )

        self.joint_state_msg = JointState()
        self.joint_states_sub_ = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_states_callback, 10
        )

        self.poses = [
            [0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
            [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
        ]

        self.poses_kuka = [
            [0.0, -1.5707963, 1.5707963, 0.0, -1.5707963, 0.0],
            [0.0, -1.5707963, 1.5707963, 0.0, 1.5707963, 0.0],
        ]

        self.poses_ur = [
            [1.5707963, -1.5707963, 1.5707963, -1.5707963, 1.5707963, 0.0],  # top
            [1.5707963, -1.5707963, 1.5707963, -1.5707963, -1.5707963, 0.0],  # home
        ]  # right

        self.poses_yaskawa = [
            [0.0, 0.509, -0.368, 0.0, 0.878, 0.0],          #cog due to force in x (neg) and torque in y (neg)
            [0.0, 0.509, -0.368, 0.0, 0.878, 1.5707963]    #cog due to force in y (neg) and torque in x (pos)
        ]

        calibration_thread = threading.Thread(target=self.calibrate_tool)
        calibration_thread.start()

    def wrench_callback(self, msg: WrenchStamped):
        with self.data_mutex:
            self.wrench_msg = msg

    def joint_states_callback(self, msg: JointState):
        with self.data_mutex:
            self.joint_state_msg = msg

    def calibrate_tool(self):
        self.get_logger().info(
            "Starting calibration process:\n"
            f"\t- robot: '{self.robot}'\n"
            f"\t- joint names: {self.joint_names}\n"
            f"\t- controller topic '{self.controller_topic}'\n"
            f"\t- wrench topic '{self.wrench_topic}'\n"
            f"\t- joint states topic '{self.joint_states_topic}'"
        )

        measurements: List[WrenchStamped] = []

        while len(self.joint_state_msg.position) == 0:
            time.sleep(0.1)

        for i in range(0, len(self.poses)):
            trajectory = JointTrajectory()
            point = JointTrajectoryPoint()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names

            if self.robot == KUKA:
                point.positions = self.poses_kuka[i]
            elif self.robot == UR:
                point.positions = self.poses_ur[i]
            elif self.robot == YASKAWA:
                point.positions = self.poses_yaskawa[i]
            else:
                point.positions = self.poses[i]

            diff = positions_diff(self.joint_state_msg.position, point.positions)
            max_angle_diff = max(diff)
            point.time_from_start.sec = (
                int(max_angle_diff / self.max_allowed_velocity_rad_per_sec) + 1
            )

            trajectory.points.append(point)
            self.trajectory_pub_.publish(trajectory)
            self.get_logger().info(
                f"Moving to position {i + 1}/{len(self.poses)}: {point.positions}"
            )

            while not positions_near(self.joint_state_msg.position, point.positions):
                """self.get_logger().info(
                    f"Current joint positions: {self.joint_state_msg.position} "
                    f"vs. target positions: {point.positions}\n"
                    "Waiting for the robot to reach the target position..."
                )"""
                time.sleep(0.1)

            # wait till robot is stable
            time.sleep(3)

            num_measurements = 500
            measurement_period = 0.01
            self.get_logger().info(
                f"Calculating average measurements for {num_measurements} samples "
                f"and {measurement_period}[sec] period"
            )
            wrench_avg = self.get_average_measurements(
                n=num_measurements, period=measurement_period
            )
            wrench_avg.header.frame_id = self.wrench_msg.header.frame_id
            #wrench_avg = self.transform_wrench(wrench_avg, self.wrench_msg.header.frame_id, "world")
            self.get_logger().info(
                f"Average measurements for position {i + 1}/{len(self.poses)}: "
                f"force: {wrench_avg.wrench.force}, torque: {wrench_avg.wrench.torque}"
            )

            measurements.append(wrench_avg)

        ### Now calculate CoG
        CoG = Vector3()
        Fg = (
            abs(measurements[0].wrench.force.x) + abs(measurements[1].wrench.force.y)
        ) / 2.0
        if Fg == 0:
            self.get_logger().error("Fg is zero, cannot calculate CoG")
            return

        # Assuming symmetry around the z-axis, calculating x, y, z components
        # TESTING
        CoG.x = -measurements[1].wrench.torque.z / Fg
        CoG.y = measurements[0].wrench.torque.z / Fg
        CoG.z = -measurements[0].wrench.torque.y / Fg

        self.CoG = CoG
        self.CoG_frame = self.wrench_msg.header.frame_id

        self.get_logger().info(
            f"Calculated Center of Gravity (CoG) from topic '{self.wrench_topic}' "
            f"for frame_id {self.wrench_msg.header.frame_id}:\n"
            f"\t- CoG_x: {CoG.x}\n"
            f"\t- CoG_y: {CoG.y}\n"
            f"\t- CoG_z: {CoG.z}\n"
            f"\t- Fg: {Fg}\n"
        )

    def get_average_measurements(self, n=500, period=0.01) -> WrenchStamped:
        total_wrench = WrenchStamped()
        for _ in range(n):
            with self.data_mutex:
                total_wrench.wrench.force.x += self.wrench_msg.wrench.force.x
                total_wrench.wrench.force.y += self.wrench_msg.wrench.force.y
                total_wrench.wrench.force.z += self.wrench_msg.wrench.force.z
                total_wrench.wrench.torque.x += self.wrench_msg.wrench.torque.x
                total_wrench.wrench.torque.y += self.wrench_msg.wrench.torque.y
                total_wrench.wrench.torque.z += self.wrench_msg.wrench.torque.z
            time.sleep(period)

        avg_wrench = WrenchStamped()
        avg_wrench.wrench.force.x = total_wrench.wrench.force.x / n
        avg_wrench.wrench.force.y = total_wrench.wrench.force.y / n
        avg_wrench.wrench.force.z = total_wrench.wrench.force.z / n
        avg_wrench.wrench.torque.x = total_wrench.wrench.torque.x / n
        avg_wrench.wrench.torque.y = total_wrench.wrench.torque.y / n
        avg_wrench.wrench.torque.z = total_wrench.wrench.torque.z / n

        return avg_wrench
    
    def broadcast_tf(self):
        if (self.CoG_frame == ""):
            return

        # Create a TransformStamped message
        t = TransformStamped()

        # Add timestamp and frame information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.CoG_frame # Source frame
        t.child_frame_id = "CoG"   # Target frame

        # Set the translation (x, y, z)
        t.transform.translation.x = self.CoG.x
        t.transform.translation.y = self.CoG.y
        t.transform.translation.z = self.CoG.z

        # Set the rotation (roll, pitch, yaw)
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.br.sendTransform(t)

    def transform_wrench(self, wrench_msg: WrenchStamped, source, target):

        if source == target:
            return wrench_msg
        
        wrench_msg.header.frame_id = target

        t = None
        while t is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    time = (self.get_clock().now() - Duration(seconds=0.1)))
            except Exception as ex:
                self.get_logger().info(
                    f'Could not transform {source} to {target}: {ex}.')
                time.sleep(0.01)
        
        rot = quat2mat([t.transform.rotation.w,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,])
        
        ### transform wrench
        forces = np.array([wrench_msg.wrench.force.x,
                           wrench_msg.wrench.force.y,
                           wrench_msg.wrench.force.z])
        torques = np.array([wrench_msg.wrench.torque.x,
                            wrench_msg.wrench.torque.y,
                            wrench_msg.wrench.torque.z])
        
        forces = np.matmul(rot, forces)            
        torques = np.matmul(rot, torques) 
        
        wrench_msg.wrench.force.x = forces[0] 
        wrench_msg.wrench.force.y = forces[1]
        wrench_msg.wrench.force.z = forces[2]
        wrench_msg.wrench.torque.x = torques[0]
        wrench_msg.wrench.torque.y = torques[1]
        wrench_msg.wrench.torque.z = torques[2]

        return wrench_msg

def main(args=None):
    rclpy.init(args=args)

    calibrate_tool_node = CalibrationNode()

    rclpy.spin(calibrate_tool_node)

    calibrate_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
