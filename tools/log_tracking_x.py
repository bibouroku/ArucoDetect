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
# 如果你的目标话题是 PoseWithCovarianceStamped，就改成：
# from geometry_msgs.msg import PoseWithCovarianceStamped


class TrackingXLogger(Node):
    def __init__(self):
        super().__init__("tracking_x_logger")

        self.declare_parameter("odom_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("csv_path", "tracking_x_log.csv")
        self.declare_parameter("fixed_target_x", 0.0)
        self.declare_parameter("use_fixed_target", False)
        self.declare_parameter("log_hz", 30.0)

        odom_topic = self.get_parameter("odom_topic").value
        target_topic = self.get_parameter("target_topic").value
        self.csv_path = os.path.expanduser(self.get_parameter("csv_path").value)
        self.fixed_target_x = float(self.get_parameter("fixed_target_x").value)
        self.use_fixed_target = bool(self.get_parameter("use_fixed_target").value)
        log_hz = float(self.get_parameter("log_hz").value)

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)

        self.start_time = time.time()
        self.latest_meas_x = None
        self.latest_target_x = None
        self.latest_meas_y = None
        self.latest_meas_z = None

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
            # 如果你的目标话题是 PoseWithCovarianceStamped，就改成：
            # self.target_sub = self.create_subscription(
            #     PoseWithCovarianceStamped,
            #     target_topic,
            #     self.target_callback,
            #     10
            # )
        else:
            self.target_sub = None
            self.latest_target_x = self.fixed_target_x

        self.file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "t",
            "x_ref",
            "x_meas",
            "e_x",
            "y_meas",
            "z_meas"
        ])

        self.timer = self.create_timer(1.0 / log_hz, self.log_once)

        self.get_logger().info(f"Logging to: {self.csv_path}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        if self.use_fixed_target:
            self.get_logger().info(f"Using fixed target x = {self.fixed_target_x}")
        else:
            self.get_logger().info(f"Target topic: {target_topic}")

    def odom_callback(self, msg: VehicleOdometry):
        self.latest_meas_x = float(msg.position[0])
        self.latest_meas_y = float(msg.position[1])
        self.latest_meas_z = float(msg.position[2])

    def target_callback(self, msg: PoseStamped):
        self.latest_target_x = float(msg.pose.position.x)

        # 如果你改成 PoseWithCovarianceStamped，就改成：
        # self.latest_target_x = float(msg.pose.pose.position.x)

    def log_once(self):
        if self.latest_meas_x is None:
            return

        if self.latest_target_x is None:
            return

        t_now = time.time() - self.start_time
        e_x = self.latest_target_x - self.latest_meas_x

        self.writer.writerow([
            t_now,
            self.latest_target_x,
            self.latest_meas_x,
            e_x,
            self.latest_meas_y,
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
    node = TrackingXLogger()

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