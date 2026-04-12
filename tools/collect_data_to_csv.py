import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from px4_msgs.msg import DataCollect
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DataCollectLogger(Node):
    def __init__(self):
        super().__init__('data_collect_logger')

        self.declare_parameter('topic_name', '/fmu/out/data_collect')
        self.declare_parameter('csv_path', 'data_collect.csv')
        self.declare_parameter('flush_every_n', 20)

        topic_name = self.get_parameter('topic_name').value
        self.csv_path = self.get_parameter('csv_path').value
        self.flush_every_n = int(self.get_parameter('flush_every_n').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            DataCollect,
            topic_name,
            self.data_collect_callback,
            qos_profile
        )
        self.row_count = 0
        self.file = None
        self.writer = None

        self._open_csv()

        self.get_logger().info(f'Subscribing topic: {topic_name}')
        self.get_logger().info(f'Writing CSV to: {os.path.abspath(self.csv_path)}')

    def _open_csv(self):
        file_exists = os.path.exists(self.csv_path)

        self.file = open(self.csv_path, 'a', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)

        if not file_exists or os.path.getsize(self.csv_path) == 0:
            header = [
                'timestamp',

                'pos_x', 'pos_y', 'pos_z',
                'vel_x', 'vel_y', 'vel_z',
                'acc_x', 'acc_y', 'acc_z',
                'ang_vel_x', 'ang_vel_y', 'ang_vel_z',

                # 注意：这里按 PX4 常见顺序保存为 w,x,y,z
                'quat_w', 'quat_x', 'quat_y', 'quat_z',

                # 虽然 DataCollect 字段可能叫 rpm，这里按控制量保存成 u0~u3
                'u0', 'u1', 'u2', 'u3',

                'voltage_v',
                'wall_time'
            ]
            self.writer.writerow(header)
            self.file.flush()

    def data_collect_callback(self, msg: DataCollect):
        # 安全取数组，防止长度异常
        def safe_get(arr, idx, default=0.0):
            return arr[idx] if idx < len(arr) else default

        row = [
            int(msg.timestamp),

            safe_get(msg.position, 0), safe_get(msg.position, 1), safe_get(msg.position, 2),
            safe_get(msg.velocity, 0), safe_get(msg.velocity, 1), safe_get(msg.velocity, 2),
            safe_get(msg.acceleration, 0), safe_get(msg.acceleration, 1), safe_get(msg.acceleration, 2),
            safe_get(msg.angular_velocity, 0), safe_get(msg.angular_velocity, 1), safe_get(msg.angular_velocity, 2),

            # 按 w,x,y,z 保存
            safe_get(msg.quaternion, 0), safe_get(msg.quaternion, 1),
            safe_get(msg.quaternion, 2), safe_get(msg.quaternion, 3),

            safe_get(msg.rpm, 0), safe_get(msg.rpm, 1),
            safe_get(msg.rpm, 2), safe_get(msg.rpm, 3),

            float(msg.voltage_v),
            datetime.now().isoformat()
        ]

        self.writer.writerow(row)
        self.row_count += 1

        if self.row_count % self.flush_every_n == 0:
            self.file.flush()
            self.get_logger().info(f'Logged {self.row_count} rows')

    def destroy_node(self):
        if self.file is not None:
            self.file.flush()
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()