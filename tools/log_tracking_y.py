#!/usr/bin/env python3

import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped


class TrackingYLogger(Node):
    def __init__(self):
        super().__init__("tracking_y_logger")

        self.declare_parameter("odom_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("csv_path", "tracking_y_log.csv")
        self.declare_parameter("fixed_target_y", 0.0)
        self.declare_parameter("use_fixed_target", False)
        self.declare_parameter("log_hz", 30.0)

        odom_topic = self.get_parameter("odom_topic").value
        target_topic = self.get_parameter("target_topic").value
        self.csv_path = os.path.expanduser(self.get_parameter("csv_path").value)
        self.fixed_target_y = float(self.get_parameter("fixed_target_y").value)
        self.use_fixed_target = bool(self.get_parameter("use_fixed_target").value)
        log_hz = float(self.get_parameter("log_hz").value)

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)

        self.start_time = time.time()
        self.latest_meas_x = None
        self.latest_meas_y = None
        self.latest_meas_z = None
        self.latest_target_y = None

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        if not self.use_fixed_target:
            target_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

            self.target_sub = self.create_subscription(
                PoseStamped,
                target_topic,
                self.target_callback,
                target_qos
            )
        else:
            self.target_sub = None
            self.latest_target_y = self.fixed_target_y

        self.file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "t",
            "y_ref",
            "y_meas",
            "e_y",
            "x_meas",
            "z_meas"
        ])

        self.timer = self.create_timer(1.0 / log_hz, self.log_once)

        self.get_logger().info(f"Logging to: {self.csv_path}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        if self.use_fixed_target:
            self.get_logger().info(f"Using fixed target y = {self.fixed_target_y}")
        else:
            self.get_logger().info(f"Target topic: {target_topic}")

    def odom_callback(self, msg: VehicleOdometry):
        self.latest_meas_x = float(msg.position[0])
        self.latest_meas_y = float(msg.position[1])
        self.latest_meas_z = float(msg.position[2])

    def target_callback(self, msg: PoseStamped):
        self.latest_target_y = float(msg.pose.position.y)

    def log_once(self):
        if self.latest_meas_y is None or self.latest_target_y is None:
            return

        t_now = time.time() - self.start_time
        e_y = self.latest_target_y - self.latest_meas_y

        self.writer.writerow([
            t_now,
            self.latest_target_y,
            self.latest_meas_y,
            e_y,
            self.latest_meas_x,
            self.latest_meas_z
        ])
        self.file.flush()

    def destroy_node(self):
        if hasattr(self, "file") and self.file:
            self.file.flush()
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingYLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()