import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2, LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SyncRos2Bag(Node):
    def __init__(self):
        super().__init__('sync_ros2bag')
        
        # Subscribers for radar and image topics
        self.image_sub1 = Subscriber(self, CompressedImage, '/camera1/color/image_raw/compressed')
        self.image_sub2 = Subscriber(self, CompressedImage, '/camera2/color/image_raw/compressed')
        self.image_sub3 = Subscriber(self, CompressedImage, '/camera3/color/image_raw/compressed')
        self.radar_sub1 = Subscriber(self, PointCloud2, '/radar_pointcloud') #/halo_radar/cropped_pointcloud
        self.radar_sub2 = Subscriber(self, LaserScan, '/scan') #/halo_radar/cropped_scan
        
        # Synchronizer with an approximate time policy
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub1, self.image_sub2, self.image_sub3, self.radar_sub1, self.radar_sub2], 
            queue_size=100, slop=10, allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Publishers for synchronized messages
        self.image_pub1 = self.create_publisher(CompressedImage, '/sync/camera1/color/image_raw/compressed', 10)
        self.image_pub2 = self.create_publisher(CompressedImage, '/sync/camera2/color/image_raw/compressed', 10)
        self.image_pub3 = self.create_publisher(CompressedImage, '/sync/camera3/color/image_raw/compressed', 10)
        self.radar_pub1 = self.create_publisher(PointCloud2, '/sync/halo_radar/cropped_pointcloud', 10)
        self.radar_pub2 = self.create_publisher(LaserScan, '/sync/halo_radar/cropped_scan', 10)
        
        self.get_logger().info("SyncRos2Bag Node Started")
    
    def sync_callback(self, image_msg1, image_msg2, image_msg3, radar_msg1, radar_msg2):
        self.get_logger().info(
            f"Publishing synchronized messages at {image_msg1.header.stamp.sec}.{image_msg1.header.stamp.nanosec}"
        )
        
        # Publish the synchronized messages
        self.image_pub1.publish(image_msg1)
        self.image_pub2.publish(image_msg2)
        self.image_pub3.publish(image_msg3)
        self.radar_pub1.publish(radar_msg1)
        self.radar_pub2.publish(radar_msg2)
        

def main(args=None):
    rclpy.init(args=args)
    node = SyncRos2Bag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
