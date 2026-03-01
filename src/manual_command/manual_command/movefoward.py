import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class MoveFoward(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
            )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.color = COLOR()

        self.start_x = None
        self.start_y = None
        self.distance_goal = 5.0
        self.reached_goal = False
        self.speed = 0.3

    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = curr_x
            self.start_y = curr_y
            self.get_logger().info(self.color.BLUE + f"Starting Position: {self.start_x}, {self.start_y}" + self.color.END)
            return

        distance_traveled = math.sqrt((curr_x - self.start_x)**2 + (curr_y - self.start_y)**2)

        vel_msg = Twist()

        if distance_traveled < self.distance_goal:
            vel_msg.linear.x = self.speed
            self.get_logger().info(self.color.YELLOW + f"Distance: {distance_traveled:.2f} m" + self.color.END)
        else:
            vel_msg.linear.x = 0.0
            if not self.reached_goal:
                self.get_logger().info(self.color.CYAN + "Goal Reached!" + self.color.END)
                self.reached_goal = True

        self.cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveFoward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()