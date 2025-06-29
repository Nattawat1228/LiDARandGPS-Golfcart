import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        
        self.publisher_ = self.create_publisher(Path, '/robot_a_path', QoSProfile(depth=10))
        
       
        self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10))

       
        self.path = Path()
        self.path.header.frame_id = 'map'  

    def odom_callback(self, msg: Odometry):
        # สร้าง PoseStamped จากข้อมูล Odometry
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'  
        pose.pose = msg.pose.pose 
        
        
        self.path.poses.append(pose)

      
        self.publisher_.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    path_recorder = PathRecorder()
    rclpy.spin(path_recorder)
    path_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

