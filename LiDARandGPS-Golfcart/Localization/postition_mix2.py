import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import pyproj
import math

def quaternion_from_yaw(yaw_rad):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qx, qy, qz, qw

def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

class PositionMixNode(Node):
    def __init__(self):
        super().__init__('position_mix_node')

        self.publisher = self.create_publisher(PoseStamped, '/position_mix', 10)
        self.create_subscription(PoseStamped, '/gnss_pose', self.gnss_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.proj = pyproj.Proj(proj='utm', zone=33, datum='WGS84')
        self.current_yaw_deg = 0.0
        self.yaw_offset = None
        self.alpha = 0.1
        self.filtered_yaw = 0.0
        self.last_yaw = None

        self.get_logger().info("✅ PositionMixNode started")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 1.0:
            return

        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw)

        if self.yaw_offset is None:
            self.yaw_offset = yaw_deg
            self.last_yaw = 0.0
            return

        yaw_corrected = normalize_angle_deg(yaw_deg - self.yaw_offset)

        # ✅ แก้ jump โดย unwrap yaw อย่างต่อเนื่อง
        delta = yaw_corrected - self.last_yaw
        delta = (delta + 180) % 360 - 180
        yaw_corrected = self.last_yaw + delta

        self.last_yaw = yaw_corrected
        self.filtered_yaw = self.alpha * yaw_corrected + (1 - self.alpha) * self.filtered_yaw
        self.current_yaw_deg = self.filtered_yaw

    def gnss_callback(self, msg: PoseStamped):
        latitude = msg.pose.position.x
        longitude = msg.pose.position.y
        utm_x, utm_y = self.proj(longitude, latitude)

        yaw_rad = math.radians(self.current_yaw_deg)
        qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = utm_x
        pose_msg.pose.position.y = utm_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.publisher.publish(pose_msg)
        self.get_logger().info(
            f"📍 Published /position_mix: x={utm_x:.2f}, y={utm_y:.2f}, yaw={self.current_yaw_deg:.2f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionMixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
