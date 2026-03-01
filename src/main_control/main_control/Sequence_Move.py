import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import time

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class Sequence_Move(Node):
    def __init__(self):
        super().__init__('sequence_move')
        self.twist_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
            )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # State variables
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.odom_initialized = False

        self.color = COLOR()
        self.run_sequence()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_initialized = True

    def move_distance(self, linear_x, target_distance):
        while not self.odom_initialized:
            rclpy.spin_once(self)

        start_pose = Point()
        start_pose.x = self.current_pose.x
        start_pose.y = self.current_pose.y

        msg = Twist()
        msg.linear.x = float(linear_x)
        self.get_logger().info(self.color.YELLOW + f"MOVING {target_distance} m..." + self.color.END)
        dist = 0.0
        while dist < target_distance:
            dist = math.sqrt((self.current_pose.x - start_pose.x)**2 +
                            (self.current_pose.y - start_pose.y)**2)
            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()

    def rotate_angle(self, speed, angle_degrees):
        while not self.odom_initialized:
            rclpy.spin_once(self)

        target_rad = math.radians(angle_degrees)
        start_yaw = self.current_yaw

        msg = Twist()
        msg.angular.z = float(speed) if angle_degrees > 0 else -float(speed)
        self.get_logger().info(self.color.YELLOW + f"ROTATING {angle_degrees} degree..." + self.color.END)

        relative_angle = 0.0
        while abs(relative_angle) < abs(target_rad):
            relative_angle = self.current_yaw - start_yaw
            while relative_angle > math.pi: relative_angle -= 2*math.pi
            while relative_angle < -math.pi: relative_angle += 2*math.pi

            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()

    def stop_robot(self):
        self.twist_pub.publish(Twist())

    def run_sequence(self):
        self.get_logger().info(self.color.CYAN + "STARTING SEQUENCE" + self.color.END)

        self.move_distance(0.3, 5.0)
        # self.rotate_angle(0.05,45)
        # self.move_distance(0.05,0.40)
        # self.rotate_angle(0.05,-38)
        self.get_logger().info(self.color.GREEN + "FINISH SEQUENCE" + self.color.END)

def main(args=None):
    rclpy.init(args=args)
    node = Sequence_Move()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()