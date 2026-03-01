import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class Compensator(Node):
    def __init__(self):
        super().__init__('compensator_node')

        self.color = COLOR()

        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            'cmd_send',
            self.data_process,
            10
        )

        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            'processed_cmd',
            10
        )

        self.threshold = 0.1

        self.get_logger().info(self.color.GREEN + "Compensator Node Started." + self.color.END)

    def data_process(self, msg):
        cmd_vel = msg.data
        vel_l = cmd_vel[0]
        vel_r = cmd_vel[1]

        if (0 < vel_l < self.threshold):
            processed_vel_l = self.threshold
        elif(-self.threshold < vel_l < 0):
            processed_vel_l = -self.threshold
        else:
            processed_vel_l = vel_l
        
        if(0 < vel_r < self.threshold):
            processed_vel_r = self.threshold
        elif(-self.threshold < vel_r < 0):
            processed_vel_r = -self.threshold
        else:
            processed_vel_r = vel_r

        processed_data = Float32MultiArray()
        processed_data.data = [processed_vel_l, processed_vel_r]

        # self.get_logger().info(f"left :{processed_vel_l} | right :{processed_vel_r}")

        self.cmd_pub.publish(processed_data)

def main(args=None):
    rclpy.init(args=args)
    node = Compensator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()