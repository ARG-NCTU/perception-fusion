import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped
from std_srvs.srv import Empty

import utm
import tf_transformations
import tf2_ros
import math
from scipy.stats import norm
import numpy as np

class UTMPoint:
    def __init__(self, easting, northing):
        self.easting = easting
        self.northing = northing

class LocalizationGPSIMU(Node):
    def __init__(self):
        super().__init__('localization_gps_imu')

        # Pose storage
        self.pose = Pose()
        self.start = False
        self.origin_set = False  # Only start localization after set_origin service call

        # Kalman filter priors (as distributions)
        self.prior_x = None
        self.prior_y = None
        self.prior_z = None
        self.prior_roll = None
        self.prior_pitch = None
        self.prior_yaw = None

        self.imu_offset = 0.0
        self.odometry = Odometry()
        self.br = tf2_ros.TransformBroadcaster(self)
        self.static_br = tf2_ros.StaticTransformBroadcaster(self)  # Static tf broadcaster

        self.current_gps = None
        self.current_imu = None

        # Publisher
        self.pub_odm = self.create_publisher(Odometry, '~/odometry', 10)

        # Subscribers
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/gps_7/navsat', self.cb_gps, 10)

        # Service to set origin
        self.srv_set_origin = self.create_service(Empty, '~/set_origin', self.cb_set_origin)

        # Broadcast static transform map -> odom
        self.broadcast_static_map_to_odom()

    def broadcast_static_map_to_odom(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        self.static_br.sendTransform(static_tf)

    def cb_gps(self, msg_gps):
        self.current_gps = msg_gps
        if hasattr(self, 'utm_orig') and self.origin_set:
            easting, northing, _, _ = utm.from_latlon(msg_gps.latitude, msg_gps.longitude)
            utm_point = UTMPoint(easting, northing)
            self.pose.position.x = utm_point.easting - self.utm_orig.easting
            self.pose.position.y = utm_point.northing - self.utm_orig.northing
            self.pose.position.z = 0.0

    def cb_imu(self, msg_imu):
        self.current_imu = msg_imu
        self.pose.orientation = msg_imu.orientation
        if hasattr(self, 'utm_orig') and self.origin_set:
            self.kalman_filter()

    def kalman_filter(self):
        q = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        yaw = yaw + math.pi / 2.0

        if not self.start:
            # Initialize priors with current measurements
            self.start = True
            self.prior_x = norm(loc=self.pose.position.x, scale=100)
            self.prior_y = norm(loc=self.pose.position.y, scale=100)
            self.prior_z = norm(loc=self.pose.position.z, scale=100)
            self.prior_roll = norm(loc=roll, scale=10)
            self.prior_pitch = norm(loc=pitch, scale=10)
            self.prior_yaw = norm(loc=yaw, scale=10)
            return

        # Prediction step
        kernel = norm(loc=0, scale=2)
        kernel_euler = norm(loc=0, scale=0.5)

        predicted_x = self.predict(self.prior_x, kernel)
        predicted_y = self.predict(self.prior_y, kernel)
        predicted_z = self.predict(self.prior_z, kernel)
        predicted_roll = self.predict(self.prior_roll, kernel_euler)
        predicted_pitch = self.predict(self.prior_pitch, kernel_euler)
        predicted_yaw = self.predict(self.prior_yaw, kernel_euler)

        # Update step
        posterior_x = self.update_con(predicted_x, self.pose.position.x, 0.05)
        posterior_y = self.update_con(predicted_y, self.pose.position.y, 0.05)
        posterior_z = self.update_con(predicted_z, self.pose.position.z, 0.05)
        posterior_roll = self.update_con(predicted_roll, roll, 0.05)
        posterior_pitch = self.update_con(predicted_pitch, pitch, 0.05)
        posterior_yaw = self.update_con(predicted_yaw, yaw, 0.05)

        # Update priors
        self.prior_x = posterior_x
        self.prior_y = posterior_y
        self.prior_z = posterior_z
        self.prior_roll = posterior_roll
        self.prior_pitch = posterior_pitch
        self.prior_yaw = posterior_yaw

        # Publish odometry
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.header.frame_id = 'map'
        self.odometry.pose.pose.position.x = posterior_x.mean()
        self.odometry.pose.pose.position.y = posterior_y.mean()
        self.odometry.pose.pose.position.z = posterior_z.mean()

        q = tf_transformations.quaternion_from_euler(0, 0, posterior_yaw.mean() + self.imu_offset)
        self.odometry.pose.pose.orientation.x = q[0]
        self.odometry.pose.pose.orientation.y = q[1]
        self.odometry.pose.pose.orientation.z = q[2]
        self.odometry.pose.pose.orientation.w = q[3]

        self.pub_odm.publish(self.odometry)

        # Broadcast dynamic transform odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odometry.pose.pose.position.x
        t.transform.translation.y = self.odometry.pose.pose.position.y
        t.transform.translation.z = self.odometry.pose.pose.position.z
        t.transform.rotation = self.odometry.pose.pose.orientation
        self.br.sendTransform(t)

    def predict(self, prior, kernel):
        return norm(loc=prior.mean() + kernel.mean(), scale=math.sqrt(prior.var() + kernel.var()))

    def measurement(self, measurementx, variance):
        return norm(loc=measurementx, scale=math.sqrt(variance))

    def gaussian_multiply(self, g1, g2):
        g1_mean, g1_var = g1.stats(moments='mv')
        g2_mean, g2_var = g2.stats(moments='mv')
        mean = (g1_var * g2_mean + g2_var * g1_mean) / (g1_var + g2_var)
        var = (g1_var * g2_var) / (g1_var + g2_var)
        return norm(loc=mean, scale=math.sqrt(var))

    def update_con(self, prior, measurementz, covariance):
        likelihood = self.measurement(measurementz, covariance)
        return self.gaussian_multiply(likelihood, prior)

    def cb_set_origin(self, request, response):
        if self.current_gps is None or self.current_imu is None:
            self.get_logger().warn('No GPS or IMU data yet!')
            return response

        # Set GPS origin
        easting, northing, _, _ = utm.from_latlon(self.current_gps.latitude, self.current_gps.longitude)
        self.utm_orig = UTMPoint(easting, northing)
        self.get_logger().info(f'[Set Origin] lat: {self.current_gps.latitude}, lon: {self.current_gps.longitude}')

        # Set IMU offset based on yaw only
        q = (self.current_imu.orientation.x,
             self.current_imu.orientation.y,
             self.current_imu.orientation.z,
             self.current_imu.orientation.w)
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.imu_offset = -(yaw + math.pi/2.0)
        self.get_logger().info(f'[Set IMU Offset] offset(rad): {self.imu_offset}')

        # Enable localization
        self.origin_set = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGPSIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
