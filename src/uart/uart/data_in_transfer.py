import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, Float32
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class DataProcessorNode(Node):
    def __init__(self):
        super().__init__('data_processor_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'uart_receive_data',
            self.data_callback,
            10
        )

        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            'processed_cmd',
            self.cmd_callback,
            10
        )

        self.physical_publish = self.create_publisher(
            Float32MultiArray,
            'physical_data',
            10
        )

        self.cur_floor_pub = self.create_publisher(
            Float32,
            'current_floor',
            10
        )

        self.arm_home_pub = self.create_publisher(
            Float32,
            'arm_home_status',
            10
        )

        self.cmd_data_publish = self.create_publisher(
            Float32MultiArray,
            'uart_send_data',
            10
        )

        self.color = COLOR()
        self.get_logger().info(self.color.GREEN + "Data Processor Node Started." + self.color.END)

    def data_callback(self, msg):
        incoming_data = msg.data

        physical_datas = Float32MultiArray()
        physical_datas.data = [incoming_data[1], incoming_data[2], incoming_data[3]]

        current_floor = Float32()
        current_floor.data = incoming_data[4]

        arm_home_status = Float32()
        arm_home_status.data = incoming_data[5]


        self.arm_home_pub.publish(arm_home_status)
        self.physical_publish.publish(physical_datas)

    def cmd_callback(self, cmd):
        incoming_data = cmd.data

        cmd_data = Float32MultiArray()
        cmd_data.data = [incoming_data[0], incoming_data[1]]

        self.cmd_data_publish.publish(cmd_data)


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
