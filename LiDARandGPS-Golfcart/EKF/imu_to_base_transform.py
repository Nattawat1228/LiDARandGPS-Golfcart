import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuToBaseLink(Node):
    def __init__(self):
        super().__init__('imu_to_base_link_transform')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # <- เปลี่ยนชื่อ topic ตรงนี้ถ้าไม่ใช่ /imu/data
            self.imu_callback,
            10
        )

    def imu_callback(self, msg: Imu):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'imu_link'
        t.child_frame_id = 'base_link'

        # Fixed position offset between imu_link and base_link
        t.transform.translation.x = 0.3
        t.transform.translation.y = 0.2
        t.transform.translation.z = 1.0

        # Use orientation from the IMU data
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToBaseLink()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

