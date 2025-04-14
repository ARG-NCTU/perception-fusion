import rclpy
from rclpy.node import Node
import sqlite3
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from pathlib import Path
from datetime import datetime
from dateutil.parser import isoparse
from dateutil import tz
import os
import shutil

class FilterROS2Bag(Node):
    def __init__(self):
        super().__init__('filter_bag')

        self.declare_parameter('input_bag', '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04')
        self.declare_parameter('output_bag', 'bags/ros2-gps-imu-10-knots-2025-04-11-12-21')
        self.declare_parameter('start_time', '2025-04-11T12:21:00+08:00')
        self.declare_parameter('end_time', '2025-04-11T12:22:59+08:00')
        self.declare_parameter('topics', None)

        self.input_bag = self.get_parameter('input_bag').value
        self.output_bag = self.get_parameter('output_bag').value
        self.start_time = self.get_parameter('start_time').value
        self.end_time = self.get_parameter('end_time').value
        self.topics = self.get_parameter('topics').value

        self.get_logger().info(f'Filtering bag from {self.start_time} to {self.end_time}...')
        
        if os.path.exists(self.output_bag):
            self.get_logger().warn(f"⚠️ Output bag directory already exists. Deleting: {self.output_bag}")
            shutil.rmtree(self.output_bag)
        
        self.filter_and_write()

    def to_nanosec(self, dt_str):
        dt = isoparse(dt_str)
        dt_utc = dt.astimezone(tz.UTC)
        return int(dt_utc.timestamp() * 1e9)

    def filter_and_write(self):
        start_ns = self.to_nanosec(self.start_time)
        end_ns = self.to_nanosec(self.end_time)

        db_path = os.path.join(self.input_bag, Path(self.input_bag).name + "_0.db3")
        if not os.path.exists(db_path):
            self.get_logger().error(f"Database not found: {db_path}")
            return

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT id, name, type FROM topics")
        topic_map = {id: (name, type) for id, name, type in cursor.fetchall()}

        
        storage_options = StorageOptions(uri=self.output_bag, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        for topic_id, (topic_name, type_name) in topic_map.items():
            if topic_name not in self.topics:
                continue

            # 建立 topic metadata 並註冊
            writer.create_topic(TopicMetadata(
                name=topic_name,
                type=type_name,
                serialization_format='cdr'
            ))

            cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?",
                (topic_id, start_ns, end_ns)
            )

            messages = cursor.fetchall()
            self.get_logger().info(f"Writing {len(messages)} messages from topic {topic_name}")

            for timestamp, data in messages:
                writer.write(topic_name, data, timestamp)

        conn.close()
        self.get_logger().info(f"✅ Finished filtering. Output saved to {self.output_bag}")

def main(args=None):
    rclpy.init(args=args)
    node = FilterROS2Bag()
    rclpy.spin_once(node, timeout_sec=0.1)  # run once and exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
