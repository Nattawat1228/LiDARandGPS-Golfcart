#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import pandas as pd
import math
import numpy as np

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

class PathYawControlTransition(Node):
    def __init__(self):
        super().__init__('path_yaw_control_transition')

        self.df = pd.read_csv('/home/mag/Downloads/smoothed_waypoints4.csv')

        self.segments = [
            (1, 395, 'straight'),
            (396, 460, 'curve'),
            (461, 583, 'straight'),
            (584, 870, 'curve'),
            (871, 1198, 'straight'),
            (1199, 1450, 'curve'),
            (1451, 1590, 'straight'),
            (1591, 1660, 'curve'),
            (1661, len(self.df) - 1, 'straight')
        ]

        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        self.angle_pub = self.create_publisher(Float32, '/cmd_angle', 10)
        self.subscription = self.create_subscription(PoseStamped, '/position_mix', self.listener_callback, 10)

        self.angle_last_straight = 0.0
        self.scale_factor = 30.0
        self.max_angle = 50.0

    def listener_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw_now = quaternion_to_yaw(msg.pose.orientation)

        wp_x = self.df['x'].values
        wp_y = self.df['y'].values
        vec_to_wp_x = wp_x - x
        vec_to_wp_y = wp_y - y

        dx_look = math.cos(yaw_now)
        dy_look = math.sin(yaw_now)
        dot_product = dx_look * vec_to_wp_x + dy_look * vec_to_wp_y

        front_indices = np.where(dot_product > 0)[0]
        if len(front_indices) == 0:
            self.get_logger().warn(" No waypoint in front → STOP")
            self.speed_pub.publish(Float32(data=0.0))
            self.angle_pub.publish(Float32(data=0.0))
            return

        dists = np.hypot(wp_x[front_indices] - x, wp_y[front_indices] - y)
        nearest_idx = front_indices[np.argmin(dists)]
        nearest_wp = self.df['wp'].iloc[nearest_idx]
        nearest_dist = dists[np.argmin(dists)]

        yaw_ref = math.radians(self.df['yaw_imu'].iloc[nearest_idx])
        yaw_err_rad = math.atan2(math.sin(yaw_now - yaw_ref), math.cos(yaw_now - yaw_ref))
        yaw_err_deg = math.degrees(yaw_err_rad)

        kind = 'UNKNOWN'
        for start, end, k in self.segments:
            if start <= nearest_idx <= end or (start > end and (nearest_idx >= start or nearest_idx <= end)):
                kind = k
                break

        if kind == 'straight':
            speed = 2.0
            if abs(yaw_err_deg) < 1.0:
                angle = 0.0
            else:
                angle = clamp(-yaw_err_deg * 0.3, -5, 5)
            self.angle_last_straight = angle

        elif kind == 'curve':
            speed = 1.0
            idx_prev = max(nearest_idx - 3, 0)
            idx_next = min(nearest_idx + 3, len(self.df) - 1)
            yaw_prev = math.radians(self.df['yaw_imu'].iloc[idx_prev])
            yaw_next = math.radians(self.df['yaw_imu'].iloc[idx_next])
            delta_yaw = math.atan2(math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev))
            steer_dir = 1.0 if delta_yaw > 0 else -1.0
            angle = clamp(steer_dir * abs(yaw_err_deg) * 0.5, -self.max_angle, self.max_angle)

        else:
            speed = 0.0
            angle = 0.0

        self.speed_pub.publish(Float32(data=float(speed)))
        self.angle_pub.publish(Float32(data=float(angle)))

        self.get_logger().info(
            f" {kind.upper()} ➤ Speed: {speed}, Angle: {angle:.1f}° | "
            f"WP: {nearest_wp} (Index: {nearest_idx}) → Dist: {nearest_dist:.2f} m | "
            f"Yaw Ref: {yaw_ref:.2f} rad | Now: {yaw_now:.2f} rad | Δyaw: {yaw_err_rad:.2f} rad ({yaw_err_deg:.1f}°)"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PathYawControlTransition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
