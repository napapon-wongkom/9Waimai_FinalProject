import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class InitialPoseManager(Node):
    def __init__(self):
        super().__init__('reset_pose')
        
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.color = COLOR()

        self.get_logger().info(self.color.GREEN + 'Initialize Pose Manager Node Started!' + self.color.END)

    def set_robot_position(self, x, y, theta_rad):
        
        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta_rad / 2.0)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.06
        msg.pose.covariance = covariance

        self.get_logger().info(self.color.GREEN + f"Robot set to x: {x}, y: {y}, th: {theta_rad}" + self.color.END)
        self.initial_pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseManager()
    node.set_robot_position(0.0, 0.0, 0.0)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()