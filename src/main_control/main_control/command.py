import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import math
import threading

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class Command(Node):
    def __init__(self):
        super().__init__('Command_Terminal')

        self.color = COLOR()
        self.get_logger().info(self.color.GREEN + "Robot Command Terminal" + self.color.END)
        self.target_floor_pub = self.create_publisher(
            Int32,
            'target_floor',
            10
        )

        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            command = input(self.color.BLUE + "Select Target Floor (1-3): " + self.color.END)

            if command in ['1', '2', '3']:
                target_floor = int(command)
                msg = Int32()
                msg.data = target_floor
                self.get_logger().info(self.color.YELLOW + f"Target set to Floor {target_floor}" + self.color.END)

                self.target_floor_pub.publish(msg)
            else:
                self.get_logger().error("Invalid input.")

def main(args=None):
    rclpy.init(args=args)
    node = Command()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()