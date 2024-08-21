#!/usr/bin/env python

from subprocess import call
from math import sqrt
import time
import threading
from typing import List

import rclpy
import rclpy.duration
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3, WrenchStamped
from std_msgs.msg import String


class CalibrationNode(Node):

    def __init__(self):
        super().__init__("calibrate_tool_node")

        ### PARAMS:
        self.tool_name = "tool0"
        self.robot = "yaskawa"
        self.robot_name = "hc20sdtp"
        self.store_to_file = False

        # switch-case on robot name to get these parameters
        self.joint_names = [
            "joint_1_s",
            "joint_2_l",
            "joint_3_u",
            "joint_4_r",
            "joint_5_b",
            "joint_6_t",
        ]
        controller_topic = "/position_trajectory_controller/joint_trajectory"
        wrench_topic = "/force_torque_sensor_broadcaster/wrench"

        self.trajectory_pub_ = self.create_publisher(
            JointTrajectory, controller_topic, 10
        )
        self.wrench_sub_ = self.create_subscription(
            WrenchStamped, wrench_topic, self.wrench_callback, 10
        )
        self.data_mutex = threading.Lock()
        self.wrench_msg = WrenchStamped()

        self.poses = [
            [0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
            [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0],
        ]

        self.poses_kuka = [
            [0.0, -1.5707963, 1.5707963, 0.0, -1.5707963, 0.0],
            [0.0, -1.5707963, 1.5707963, 0.0, 1.5707963, 0.0],
            [0.0, -1.5707963, 1.5707963, 0.0, 0.0, 0.0],
        ]

        self.poses_ur = [
            [1.5707963, -1.5707963, 1.5707963, -1.5707963, 1.5707963, 0.0],  # top
            [1.5707963, -1.5707963, 1.5707963, -1.5707963, -1.5707963, 0.0],  # home
            [1.5707963, -1.5707963, 1.5707963, -1.5707963, 0.0, 0.0],
        ]  # right

        self.poses_yaskawa = [
            [0.0, 0.509, -0.368, 0.0, 0.878, 0.0],
            [0.0, 0.509, -0.368, 0.0, 0.878, 1.5707963],
            [0.0, 0.509, -0.368, 0.0, 0.878, -1.5707963],
        ]

        calibration_thread = threading.Thread(target=self.calibrate_tool)
        calibration_thread.start()

    def wrench_callback(self, msg):
        with self.data_mutex:
            self.wrench_msg = msg

    def calibrate_tool(self):
        measurements: List[WrenchStamped] = []

        for i in range(0, len(self.poses)):
            trajectory = JointTrajectory()
            point = JointTrajectoryPoint()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names

            point.time_from_start.sec = 2
            if self.robot == "kuka":
                point.positions = self.poses_kuka[i]
            elif self.robot == "ur":
                point.positions = self.poses_ur[i]
            elif self.robot == "yaskawa":
                point.positions = self.poses_yaskawa[i]
            else:
                point.positions = self.poses[i]

            trajectory.points.append(point)
            self.trajectory_pub_.publish(trajectory)
            self.get_logger().info(
                f"Moving to position {i + 1}/{len(self.poses)}: {point.positions}"
            )

            time.sleep(point.time_from_start.sec * 2)

            num_measurements = 500
            measurement_period = 0.01
            self.get_logger().info(
                f"Calculating average measurements for {num_measurements} samples "
                f"and {measurement_period}[sec] period"
            )
            wrench_avg = self.get_average_measurements(
                n=num_measurements, period=measurement_period
            )
            self.get_logger().info(
                f"Average measurements for position {i + 1}/{len(self.poses)}: "
                f"force: {wrench_avg.wrench.force}, torque: {wrench_avg.wrench.torque}"
            )

            measurements.append(wrench_avg)

        ### Now calculate CoG
        CoG = Vector3()
        Fg = (
            abs(measurements[0].wrench.force.z) + abs(measurements[1].wrench.force.z)
        ) / 2.0
        if Fg == 0:
            self.get_logger().error("Fg is zero, cannot calculate CoG")
            return

        # Assuming symmetry around the z-axis, calculating x, y, z components
        # TESTING
        CoG.x = -measurements[2].wrench.torque.y / Fg
        CoG.y = measurements[2].wrench.torque.x / Fg

        CoG.z = (
            sqrt(
                measurements[2].wrench.torque.x ** 2
                + measurements[2].wrench.torque.y ** 2
            )
        ) / Fg

        self.get_logger().info(
            f"Calculated Center of Gravity (CoG) for {self.tool_name}:\n"
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


def main(args=None):
    rclpy.init(args=args)

    calibrate_tool_node = CalibrationNode()

    rclpy.spin(calibrate_tool_node)

    calibrate_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
