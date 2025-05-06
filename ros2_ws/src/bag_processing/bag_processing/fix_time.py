import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import importlib
from std_msgs.msg import Header

class FixTime(Node):
    def __init__(self):
        super().__init__('fix_time')

        # Correct: declare 'topics' as STRING_ARRAY type
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING_ARRAY)

        # Get topics from parameter
        topic_list = self.get_parameter('topics').get_parameter_value().string_array_value
        self.get_logger().info(f'Time Fixer Topics: {topic_list}')

        self.subs = []
        self.pubs = []

        # Now, for each topic
        for topic_name in topic_list:
            self.get_logger().info(f"Processing topic: {topic_name}")

            # Dynamically query its type
            topic_info = dict(self.get_topic_names_and_types())
            if topic_name not in topic_info:
                self.get_logger().warn(f"Topic {topic_name} not found. Skipping.")
                continue

            msg_type_str = topic_info[topic_name][0]
            pkg_name = msg_type_str.split('/')[0]
            msg_name = msg_type_str.split('/')[-1]
            msg_module = importlib.import_module(f'{pkg_name}.msg')
            msg_class = getattr(msg_module, msg_name)

            pub = self.create_publisher(msg_class, f"{topic_name}_fixed", QoSProfile(depth=10))
            self.pubs.append(pub)

            sub = self.create_subscription(
                msg_class,
                topic_name,
                lambda msg, pub=pub: self.callback(msg, pub),
                QoSProfile(depth=10)
            )
            self.subs.append(sub)

    def callback(self, msg, pub):
        if hasattr(msg, 'header') and isinstance(msg.header, Header):
            msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixTime()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
