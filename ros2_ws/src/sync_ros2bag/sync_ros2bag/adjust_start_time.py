import rclpy
import rosbag2_py
import os
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

class SyncRos2Bag(Node):
    def __init__(self, input_bag, output_bag, offset_sec):
        super().__init__('sync_ros2bag')
        self.input_bag = input_bag
        self.output_bag = output_bag
        self.offset_sec = offset_sec
        
        # Create output directory if it does not exist
        os.makedirs(os.path.dirname(self.output_bag), exist_ok=True)
    
    def adjust_timestamp(self, msg):
        """Adjust ROS 2 message timestamps"""
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            msg.header.stamp.sec -= self.offset_sec
            if msg.header.stamp.sec < 0:
                msg.header.stamp.sec = 0  # Prevent negative timestamps
        return msg
    
    def convert_rosbag(self):
        """Read the old rosbag, modify timestamps, and save to a new rosbag"""
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.input_bag, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        
        reader.open(storage_options, converter_options)
        
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=self.output_bag, storage_id='sqlite3'), converter_options)
    
        # Read topic information
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        for topic in reader.get_all_topics_and_types():
            writer.create_topic(topic)
        
        # Read and adjust data
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(topic_types[topic])
            msg = deserialize_message(data, msg_type)
            
            # Adjust timestamp
            new_msg = self.adjust_timestamp(msg)
            new_data = serialize_message(new_msg)
            
            writer.write(topic, new_data, t - self.offset_sec * 1_000_000_000)  # ROS 2 time unit is nanoseconds
    
        self.get_logger().info(f"New Bag `{self.output_bag}` created, all timestamps adjusted by {self.offset_sec} seconds!")


def main():
    rclpy.init()
    input_bag = "bags/20241129_cropped/processed_rosbag_halo_20241205-145227/processed_rosbag_halo_20241205-145227_0.db3"
    output_bag = "bags/20241129_cropped_fixed/processed_rosbag_halo_20241205-145227"
    offset_sec = 518780  # Adjust time backward

    sync_bag = SyncRos2Bag(input_bag, output_bag, offset_sec)
    sync_bag.convert_rosbag()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
