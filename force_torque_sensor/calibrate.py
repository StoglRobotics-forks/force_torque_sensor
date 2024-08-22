#!/usr/bin/env python
from math import sqrt
import time
import threading

import rclpy
import rclpy.duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3, WrenchStamped
from force_torque_sensor.calibration_node_parameters import calibration_node
class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')
        
        param_listener = calibration_node.ParamListener(self)
        params = param_listener.get_params()
        self.joint_names = params.joints
        self.robot = params.robot
        self.gravity_axis_in_sensor_frame = params.ft_sensor.up

        self.trajectory_pub_ = self.create_publisher(JointTrajectory, params.ft_sensor.controller_topic, 10)
        self.wrench_sub_ = self.create_subscription(WrenchStamped, params.ft_sensor.wrench_topic, self.wrench_callback, 10)
        self.wrench_msg = WrenchStamped()
        
        self.poses = [[0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
                      [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0]]

        self.poses_kuka = [[0.0, -1.5707963, 1.5707963, 0.0, -1.5707963, 0.0],
                           [0.0, -1.5707963, 1.5707963, 0.0, 1.5707963, 0.0],
                           [0.0, -1.5707963, 1.5707963, 0.0, 0.0, 0.0]]

        self.poses_ur = [[1.5707963, -1.5707963, 1.5707963, -1.5707963, 1.5707963, 0.0],  # top
                         [1.5707963, -1.5707963, 1.5707963, -1.5707963, -1.5707963, 0.0], # home
                         [1.5707963, -1.5707963, 1.5707963, -1.5707963, 0.0, 0.0]]        # right
    
        self.poses_yaskawa = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963],     
                              [0.0, 0.0, 0.0, 0.0, 0.0, -1.5707963]] 
        
        calibration_thread = threading.Thread(target=self.calibrate)
        calibration_thread.start()

    def wrench_callback(self, msg):
        self.wrench_msg = msg
        return
    
    def calibrate(self):
        measurement = []

        for i in range(0,len(self.poses)):  
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
            self.get_logger().info("Going to position: " + str(point.positions))
            
            time.sleep(10.0)
            
            self.get_logger().info("Calculating tool force...")

            wrench_avg = self.get_average_measurements(n=500, period=0.01)

            measurement.append(wrench_avg)

        ### Calculate CoG
        CoG = Vector3()
        Fg = 0.0
        if self.gravity_axis_in_sensor_frame == "z":
            Fg = (abs(measurement[0].wrench.force.z) + abs(measurement[1].wrench.force.z))/2.0
            CoG.z = (sqrt(measurement[2].wrench.torque.x*measurement[2].wrench.torque.x + measurement[2].wrench.torque.y*measurement[2].wrench.torque.y)) / Fg
        elif self.gravity_axis_in_sensor_frame == "x":
            Fg = (abs(measurement[0].wrench.force.x) + abs(measurement[1].wrench.force.x))/2.0
            CoG.z = (sqrt(measurement[2].wrench.torque.z*measurement[2].wrench.torque.z + measurement[2].wrench.torque.y*measurement[2].wrench.torque.y)) / Fg
        elif self.gravity_axis_in_sensor_frame == "y":
            Fg = (abs(measurement[0].wrench.force.y) + abs(measurement[1].wrench.force.y))/2.0
            CoG.z = (sqrt(measurement[2].wrench.torque.z*measurement[2].wrench.torque.z + measurement[2].wrench.torque.x*measurement[2].wrench.torque.x)) / Fg
        else:
            raise ValueError("Non-existent axis value for [gravity_axis_in_sensor_frame]")
        
        self.get_logger().info("\n")
        self.get_logger().info("Calculated parameters for [" + self.wrench_msg.header.frame_id + "]:\n"
                              +  "CoG_x: " + str(CoG.x) + "\n"
                              +  "CoG_y: " + str(CoG.y) + "\n"
                              +  "CoG_z: " + str(CoG.z) + "\n"
                              +  "Fg: " + str(Fg) + "\n")


        ### Calculate offset
        offset_wrench = self.wrench_msg
        for i in range(1, len(measurement)): # skip first measurement
            offset_wrench.wrench.force.x += measurement[i].wrench.force.x
            offset_wrench.wrench.force.y += measurement[i].wrench.force.y
            offset_wrench.wrench.force.z += measurement[i].wrench.force.z
            offset_wrench.wrench.torque.x += measurement[i].wrench.torque.x
            offset_wrench.wrench.torque.y += measurement[i].wrench.torque.y
            offset_wrench.wrench.torque.z += measurement[i].wrench.torque.z
        offset_wrench.wrench.force.x /= len(self.poses)
        offset_wrench.wrench.force.y /= len(self.poses)
        offset_wrench.wrench.force.z /= len(self.poses)
        offset_wrench.wrench.torque.x /= len(self.poses)
        offset_wrench.wrench.torque.y /= len(self.poses)
        offset_wrench.wrench.torque.z /= len(self.poses)  
        
        self.get_logger().info("Calculated offset:\n"
                              +  "Fx: " + str(offset_wrench.wrench.force.x) + "\n"
                              +  "Fy: " + str(offset_wrench.wrench.force.y) + "\n"
                              +  "Fz: " + str(offset_wrench.wrench.force.z) + "\n"
                              +  "Mx: " + str(offset_wrench.wrench.torque.x) + "\n"
                              +  "My: " + str(offset_wrench.wrench.torque.y) + "\n"
                              +  "Mz: " + str(offset_wrench.wrench.torque.z))

        return

    def get_average_measurements(self, n=500, period=0.01):
        wrench_average = WrenchStamped()
        
        last_msg_time = float(self.get_clock().now().to_msg().nanosec/10e9)
        i = 0
        while i < n:
            msg_time = float(self.wrench_msg.header.stamp.nanosec/10e9)
            if abs(msg_time - last_msg_time) > period:
                wrench_average.wrench.force.x += self.wrench_msg.wrench.force.x
                wrench_average.wrench.force.y += self.wrench_msg.wrench.force.y
                wrench_average.wrench.force.z += self.wrench_msg.wrench.force.z
                wrench_average.wrench.torque.x += self.wrench_msg.wrench.torque.x
                wrench_average.wrench.torque.y += self.wrench_msg.wrench.torque.y
                wrench_average.wrench.torque.z += self.wrench_msg.wrench.torque.z
                i += 1
        
        wrench_average.wrench.force.x  /= n
        wrench_average.wrench.force.y  /= n
        wrench_average.wrench.force.z  /= n
        wrench_average.wrench.torque.x /= n
        wrench_average.wrench.torque.y /= n
        wrench_average.wrench.torque.z /= n           
        
        return wrench_average

def main(args=None):
    rclpy.init(args=args)

    calibration_node = CalibrationNode()

    rclpy.spin(calibration_node)

    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()