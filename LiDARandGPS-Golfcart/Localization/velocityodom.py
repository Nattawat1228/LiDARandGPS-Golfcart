import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import tf_transformations

class OdomFromVelYaw(Node):
    def __init__(self):
        super().__init__('odom_from_vel_yaw')

        # ตัวแปรสถานะ
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()

        # Subscriber
        self.create_subscription(Vector3Stamped, '/filter/velocity', self.velocity_callback, 10)
        self.create_subscription(Float32, '/IMU_yaw', self.yaw_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer 50 Hz
        self.create_timer(0.02, self.update_odometry)

    def velocity_callback(self, msg):
        threshold = 0.01  # กำจัด noise ความเร็วต่ำกว่า 1 cm/s
        self.vx = msg.vector.x if abs(msg.vector.x) >= threshold else 0.0
        self.vy = msg.vector.y if abs(msg.vector.y) >= threshold else 0.0

    def yaw_callback(self, msg):
        self.yaw = msg.data  # ✅ ไม่กลับด้าน yaw

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.vx == 0.0 and self.vy == 0.0:
            return

        # ✅ ใช้ vx, vy ตรง ๆ
        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.x += dx
        self.y += dy

        # สร้าง quaternion จาก yaw
        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)

        # สร้าง Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

        # log ค่าดู realtime
        self.get_logger().info(
            f"[odom] dt={dt:.3f}s vx={self.vx:.2f} vy={self.vy:.2f} yaw={math.degrees(self.yaw):.1f}° → x={self.x:.2f}, y={self.y:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromVelYaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
