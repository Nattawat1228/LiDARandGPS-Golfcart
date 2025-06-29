import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

class PoseToArrayBridge(Node):
    def __init__(self):
        super().__init__('pose_to_array_bridge')
        self.sub = self.create_subscription(PoseStamped, '/position_mix', self.callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/position_array', 10)
        self.get_logger().info("✅ PoseToArrayBridge node started. Waiting for /position_pose...")

    def callback(self, msg):
        array_msg = Float64MultiArray()
        array_msg.data = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z  # theta
        ]
        self.pub.publish(array_msg)
        self.get_logger().info(f"🔄 Bridged to /position_array: x={array_msg.data[0]:.2f}, y={array_msg.data[1]:.2f}, theta={array_msg.data[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseToArrayBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
