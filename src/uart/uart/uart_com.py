import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class UARTBridgeNode(Node):
    def __init__(self):
        super().__init__('UART_bridge_node')

        # // Initialize UART Serial
        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)

        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'uart_receive_data', 
            10
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'uart_send_data',
            self.send_callback,
            10
        )

        self.counter = 0
        self.rx_buffer = bytearray()

        self.last_send = []
        self.last_receive = []

        self.init_time = time.time()

        self.header_byte = 0xAA
        self.num_floats = 13

        self.payload_size = self.num_floats * 4

        self.total_packet_size = self.payload_size + 1 ### array_size x 4 bytes + 1

        self.timer = self.create_timer(0.01, self.receive_callback)
        # self.terminal_timer = self.create_timer(0.01, self.update_terminal)

        self.color = COLOR()
        self.get_logger().info(self.color.GREEN + "UART Bridge Node Started." + self.color.END)

    def update_terminal(self):
        """
            Handles terminal output formatting.
        """
        format_receive = [round(x, 2) for x in self.last_receive]
        status_count = ((self.counter // 50) % 4)
        loading = ['\u2500', '\\', '|', '/']
        GREEN = "\033[92m"
        STOP = "\033[0m"
        print("\033[F\033[F\033[F", end="")
        print(f"{GREEN}[STATUS] UART Node is running:{STOP} {loading[status_count]}\033[K")
        print(f"{GREEN}[INFO] Timestamp:{STOP} {(time.time()-self.init_time):.2f} sec\033[K")
        print(f"{GREEN}[STATUS] Data Sent:{STOP} array('i', {self.last_send})\033[K")
        print(f"{GREEN}[STATUS] Data Received:{STOP} array('i', {self.last_receive})\033[K", end="", flush=True)
        self.counter += 1

    def send_callback(self, msg):
        """
            Triggered when a message is received to be sent over UART.
        """
        try:
            num_floats = len(msg.data)
            format_string = f'<B{num_floats}f'
            binary_data = struct.pack(format_string, 0xAA, *msg.data)

            self.ser.write(binary_data)
            self.ser.flush()
            self.last_send = list(msg.data)
        except Exception as e:
            self.get_logger().error(f"UART send Error: {e}")

    def receive_callback(self):
        """
            Checks the serial buffer for incoming bytes.
        """
        if self.ser.in_waiting > 0:
            self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

            while len(self.rx_buffer) >= self.total_packet_size:
                if self.rx_buffer[0] == self.header_byte: # Check Starter bit
                    packet = self.rx_buffer[1 : self.total_packet_size]

                    try:
                    # Unpack data
                        self.last_receive = list(struct.unpack(f'<{self.num_floats}f', packet))

                        msg = Float32MultiArray()
                        msg.data = self.last_receive
                        self.publisher_.publish(msg)
                    except struct.error as e:
                        self.get_logger().error(f"Unpack error: {e}")

                    del self.rx_buffer[:self.total_packet_size]
                else:
                    del self.rx_buffer[0]

def main(args=None):
    rclpy.init(args=args)
    node = UARTBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()