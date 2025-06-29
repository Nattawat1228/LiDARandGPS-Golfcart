import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import pandas as pd
import numpy as np
from scipy.spatial import KDTree

class WaypointIdentifier(Node):
    def __init__(self):
        super().__init__('waypoint_identifier')

        df = pd.read_csv('/home/mag/Downloads/smoothed_waypoints4.csv')
        self.wp_coords = df[['x', 'y']].values
        self.wp_names = ['wp' + str(i + 1) for i in range(len(self.wp_coords))]
        self.kdtree = KDTree(self.wp_coords)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/position_array',
            self.position_callback,
            10
        )

        self.publisher = self.create_publisher(String, '/waypoint', 10)
        self.get_logger().info("✅ WaypointIdentifier node started. Waiting for /position_array...")

    def position_callback(self, msg):
        x, y = msg.data[0], msg.data[1]
        dist, idx = self.kdtree.query([x, y])
        closest_wp = self.wp_names[idx]

        msg_out = String()
        msg_out.data = f'wp: {closest_wp}'
        self.publisher.publish(msg_out)
        self.get_logger().info(f'📌 Closest waypoint: {closest_wp} (distance: {dist:.2f} m)')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
