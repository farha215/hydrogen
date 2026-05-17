#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrailVisualizer(Node):
    """
    Simple node to turn an Odometry stream into a Path for RViz visualization.
    Does NOT use SLAM or a global map.
    """
    def __init__(self):
        super().__init__('trail_visualizer')
        self.declare_parameter('use_sim_time', True)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/traversed_path', 10)
        self.path = Path()
        self.get_logger().info("Trail Visualizer started (SLAM-free).")

    def odom_callback(self, msg):
        # We use the frame provided by the odom source (usually 'odom' or 'world')
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = msg.header.frame_id
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        
        # Keep the last 10 minutes of trail (approx 3000 points @ 5Hz)
        if len(self.path.poses) > 3000:
            self.path.poses.pop(0)
            
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = TrailVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
