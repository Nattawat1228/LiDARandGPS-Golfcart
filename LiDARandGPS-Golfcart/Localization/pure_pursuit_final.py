import numpy as np
from scipy.spatial import KDTree
import pandas as pd
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from time import time
from datetime import datetime
import csv
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        try:
            df = pd.read_csv('/home/mag/Downloads/smoothed_waypoints4.csv')
            self.path_points = df[['x', 'y']].values
            self.path_tree = KDTree(self.path_points)
        except Exception as e:
            self.get_logger().error(f"Failed to load path CSV: {e}")
            self.path_points = np.empty((0, 2))
            self.path_tree = None

        self.position_pose = None
        self.amcl_pose = None

        self.position_pose_sub = self.create_subscription(PoseStamped, '/position_pose', self.position_pose_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.yaw_sub = self.create_subscription(Float32, '/IMU_yaw', self.yaw_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/pure_pursuit', 10)

        self.car_position = np.array([0.0, 0.0])
        self.car_theta = 0.0
        self.prev_position = None

        self.wheelbase = 1.8
        self.v = 3.0
        self.prev_theta = 0.0
        self.alpha_filter = 0.6
        self.max_turn_angle = np.radians(50.0)
        self.min_turn_clip = np.radians(1.0)
        self.last_publish_time = time()
        self.publish_interval = 0.2

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        try:
            self.log_file = open(f'/home/mag/pure_pursuit_log_{timestamp}.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow([
                'time', 'car_x', 'car_y', 'car_theta_deg',
                'lookahead_x', 'lookahead_y', 'alpha_deg',
                'steering_angle_deg', 'velocity', 'cte', 'nearest_index'
            ])
        except Exception as e:
            self.get_logger().error(f"Failed to open log file for writing: {e}")
            self.log_file = None
            self.csv_writer = None

        self.get_logger().info(" PurePursuitNode started. Waiting for /position_pose and /IMU_yaw...")

    def position_pose_callback(self, msg):
        self.position_pose = msg
        self.try_run_pure_pursuit()

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
        self.try_run_pure_pursuit()

    def yaw_callback(self, msg):
        self.car_theta = msg.data

    def try_run_pure_pursuit(self):
        if self.path_tree is None or len(self.path_points) == 0:
            self.get_logger().warn("Path data not loaded properly.")
            return

        now = time()
        if now - self.last_publish_time < self.publish_interval:
            return
        self.last_publish_time = now

        # Use the correct pose depending on current index
        index_guess = self.path_tree.query(self.car_position)[1] if self.car_position is not None else 0
        is_curve = 411 <= index_guess <= 465 or 616 <= index_guess <= 860 or 1226 <= index_guess <= 1447 or 1617 <= index_guess <= 1679

        # Choose pose source
        pose_msg = self.amcl_pose if is_curve and self.amcl_pose else self.position_pose
        if not pose_msg:
            return

        if hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'pose'):
            self.car_position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        else:
            self.car_position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y])

        _, nearest_index = self.path_tree.query(self.car_position)

        lookahead_distance = 4.0 if not is_curve else 3.5

        lookahead_point = None
        heading_vec = np.array([np.cos(self.car_theta), np.sin(self.car_theta)])
        for i in range(nearest_index + 5, nearest_index + 50):
            i_mod = i % len(self.path_points)
            candidate = self.path_points[i_mod]
            dist = np.linalg.norm(candidate - self.car_position)
            if dist >= lookahead_distance:
                car_to_candidate = candidate - self.car_position
                if np.dot(car_to_candidate, heading_vec) > 0:
                    lookahead_point = candidate
                    break

        if lookahead_point is None:
            fallback_index = (nearest_index + 10) % len(self.path_points)
            lookahead_point = self.path_points[fallback_index]

        dx = lookahead_point[0] - self.car_position[0]
        dy = lookahead_point[1] - self.car_position[1]
        angle_to_point = np.arctan2(dy, dx)
        alpha = angle_to_point - self.car_theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        curvature = 2 * np.sin(alpha) / lookahead_distance
        steering_angle = np.arctan(self.wheelbase * curvature)
        steering_angle_clipped = np.clip(steering_angle, -self.max_turn_angle, self.max_turn_angle)

        smoothed_theta = self.alpha_filter * steering_angle_clipped + (1 - self.alpha_filter) * self.prev_theta

        if not is_curve and abs(smoothed_theta) < self.min_turn_clip:
            self.get_logger().info(" Zero angular.z to keep straight line")
            smoothed_theta = 0.0

        self.prev_theta = smoothed_theta
        self.v = 1.8 if is_curve else 3.0

        car_to_point = lookahead_point - self.car_position
        cte = np.cross(car_to_point, heading_vec)

        if abs(cte) > 7.0:
            self.v = max(0.5, self.v * 0.5)
            steering_angle_clipped = np.clip(steering_angle_clipped, -np.pi / 4, np.pi / 4)
            cmd_msg = Twist()
            cmd_msg.linear.x = self.v
            cmd_msg.angular.z = steering_angle_clipped
            self.cmd_vel_pub.publish(cmd_msg)
            return

        if self.csv_writer:
            self.csv_writer.writerow([
                now,
                self.car_position[0], self.car_position[1],
                np.degrees(self.car_theta),
                lookahead_point[0], lookahead_point[1],
                np.degrees(alpha),
                np.degrees(steering_angle_clipped),
                self.v,
                cte,
                nearest_index
            ])
            if self.log_file:
                self.log_file.flush()

        self.get_logger().info(f" Index: {nearest_index} | Curve: {is_curve} | v: {self.v:.2f} | theta: {np.degrees(smoothed_theta):.2f}°")

        cmd_msg = Twist()
        cmd_msg.linear.x = self.v
        cmd_msg.angular.z = np.clip(smoothed_theta, -self.max_turn_angle, self.max_turn_angle)
        self.cmd_vel_pub.publish(cmd_msg)

    def destroy_node(self):
        try:
            if self.log_file and not self.log_file.closed:
                self.log_file.close()
        except Exception as e:
            self.get_logger().warn(f"Failed to close log file: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
