#!/usr/bin/env python

from dataclasses import dataclass
from enum import Enum
from math import sqrt
import time
import threading
from typing import List

import rclpy
import rclpy.duration
from rclpy.time import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3, WrenchStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.executors import MultiThreadedExecutor


KUKA = "kuka"
UR = "ur"
YASKAWA = "yaskawa"


def positions_diff(a: List[float], b: List[float]) -> List[float]:
    return [abs(a[i] - b[i]) for i in range(len(a))]


def positions_near(a: List[float], b: List[float], eps=0.01) -> bool:
    return all([diff < eps for diff in positions_diff(a, b)])


class CalibrationNode(Node):

    def __init__(self):
        super().__init__("calibrate_tool_node")

        ### PARAMS:
        self.robot = self.declare_parameter("robot", "yaskawa").value
        self.joint_names = self.declare_parameter(
            "joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
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
            "max_allowed_velocity_rad_per_sec", 0.3
        ).value

        self.data_mutex = threading.Lock()

        # self.trajectory_pub_ = self.create_publisher(
        #     JointTrajectory, self.controller_topic, 10
        # )

        self.action_client_callback_group = MutuallyExclusiveCallbackGroup()
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=self.action_client_callback_group,
        )

        self.wrench_msg = WrenchStamped()
        self.wrench_sub_ = self.create_subscription(
            WrenchStamped, self.wrench_topic, self.wrench_callback, 10
        )

        # self.joint_state_msg = JointState()
        self.current_joint_positions = None
        self.joint_states_sub_ = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_states_callback, 10
        )

        # self.poses = [
        #     [0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
        #     [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
        #     [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0],
        # ]

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
            # in degrees: [0.0, 29.2, -21.1, 0.0, 50.2, 0.0]
            # [0.0, 0.509, -0.368, 0.0, 0.878, -1.5707963],
            # in degrees: [0.0, 29.2, -21.1, 0.0, 50.2, 90.0]
            [0.0, 0.509, -0.368, 0.0, 0.878, 1.5707963],
            # in degrees: [0.0, 29.2, -21.1, 0.0, 50.2, -90.0]
            [0.0, 0.509, -0.368, 0.0, 0.878, -1.5707963],
        ]

        if self.robot == KUKA:
            self.poses = self.poses_kuka
        elif self.robot == UR:
            self.poses = self.poses_ur
        elif self.robot == YASKAWA:
            self.poses = self.poses_yaskawa
        else:
            raise ValueError(f"Unknown robot '{self.robot}'")

        calibration_thread = threading.Thread(target=self.calibrate_tool)
        calibration_thread.start()

    def wrench_callback(self, msg: WrenchStamped):
        with self.data_mutex:
            self.wrench_msg = msg

    def joint_states_callback(self, msg: JointState):
        with self.data_mutex:
            for i in range(len(self.joint_names)):
                if self.joint_names[i] not in msg.name:
                    self.get_logger().error(
                        f"Joint '{self.joint_names[i]}' not found in joint states message"
                    )
                    return

            for i in range(len(self.joint_names)):
                if self.current_joint_positions is None:
                    self.current_joint_positions = [0] * len(self.joint_names)

                idx = msg.name.index(self.joint_names[i])
                self.current_joint_positions[i] = msg.position[idx]

    def send_joint_trajectory(
        self,
        joint_positions: List[float],
        duration: float = 5.0,
    ) -> bool:
        self.get_logger().info(
            f"Sending joint trajectory positions {joint_positions} in {duration} seconds"
            f" to controller '{self.controller_topic}'"
        )
        self.execute_trajectory_client.wait_for_server()
        goal = ExecuteTrajectory.Goal()
        goal.controller_names = ["position_trajectory_controller"]
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        start_point = JointTrajectoryPoint()

        while self.current_joint_positions is None:
            self.get_logger().info("Waiting for joint states to be published...")
            time.sleep(0.1)

        start_point.positions = self.current_joint_positions
        start_point.time_from_start = Duration(seconds=0.0).to_msg()
        traj.points.append(start_point)
        end_point = JointTrajectoryPoint()
        end_point.positions = joint_positions
        end_point.time_from_start = Duration(seconds=duration).to_msg()
        traj.points.append(end_point)
        goal.trajectory.joint_trajectory = traj
        exec_result: ExecuteTrajectory.Result = (
            self.execute_trajectory_client.send_goal(goal).result
        )
        if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Joint trajectory execution succeeded.")
            return True
        else:
            self.get_logger().error(
                f"Joint trajectory execution failed with error code: {exec_result.error_code.val}"
                f" and message: {exec_result.error_code.message}"
            )
            return False

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

        if self.current_joint_positions is None:
            self.get_logger().info("Waiting for joint states to be published...")
            while self.current_joint_positions is None:
                time.sleep(0.1)

        for i in range(len(self.poses)):
            diff = positions_diff(self.current_joint_positions, self.poses[i])
            max_angle_diff = max(diff)
            time_from_start_sec = (
                int(max_angle_diff / self.max_allowed_velocity_rad_per_sec) + 1
            )

            self.get_logger().info(
                f"Moving to position {i + 1}/{len(self.poses)}: {self.poses[i]}"
                f"\n\t- current joint positions: {self.current_joint_positions}"
            )

            if not self.send_joint_trajectory(
                joint_positions=self.poses[i], duration=time_from_start_sec
            ):
                self.get_logger().error(
                    f"Failed to send joint trajectory to position {i + 1}/{len(self.poses)}"
                )
                exit(0)

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
            abs(measurements[0].wrench.force.y) + abs(measurements[1].wrench.force.y)
        ) / 2.0
        if Fg == 0:
            self.get_logger().error("Fg is zero, cannot calculate CoG")
            return

        # Assuming symmetry around the z-axis, calculating x, y, z components
        CoG.x = 0.0  # assuming symmetry around the z-axis
        CoG.y = measurements[0].wrench.torque.z / Fg
        CoG.z = -measurements[0].wrench.torque.y / Fg
        # CoG.x = -measurements[1].wrench.torque.x / Fg
        # CoG.y = measurements[2].wrench.torque.x / Fg

        # CoG.z = (
        #     sqrt(
        #         measurements[2].wrench.torque.x ** 2
        #         + measurements[2].wrench.torque.y ** 2
        #     )
        # ) / Fg

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


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    calibrate_tool_node = CalibrationNode()

    try:
        rclpy.spin(calibrate_tool_node, executor)
    except KeyboardInterrupt:
        pass

    calibrate_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
