import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RelayTfNode(Node):
    def __init__(self):
        super().__init__('relay_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.relay_tf)  # 每0.1秒執行一次

    def relay_tf(self):
        try:
            # 嘗試取得 gps_7 -> vessel_base_link 的 transform
            old_tf: TransformStamped = self.tf_buffer.lookup_transform(
                'gps_7', 'vessel_base_link', rclpy.time.Time()
            )

            # 建立新的 transform: gps_7 -> base_link
            new_tf = TransformStamped()
            new_tf.header.stamp = self.get_clock().now().to_msg()  # ✅ 用現在時間
            new_tf.header.frame_id = 'gps_7'
            new_tf.child_frame_id = 'base_link'
            new_tf.transform = old_tf.transform  # ✅ 複製 transform 資訊

            self.broadcaster.sendTransform(new_tf)

        except Exception as e:
            self.get_logger().warn(f'Failed to relay TF: {e}')

def main():
    rclpy.init()
    node = RelayTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
