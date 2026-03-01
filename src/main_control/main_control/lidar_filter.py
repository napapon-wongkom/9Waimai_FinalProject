import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Assuming LIDAR data is of type LaserScan
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class LidarFilter(Node):
    """
        A ROS2 node that filters out LIDAR readings in specified angular ranges by setting them to infinity.
    """
    def __init__(self):
        """
            Initialize the LidarFilter node, set up subscriptions and publishers.
            - Subscribes to '/scan' topic for incoming LIDAR data.
            - Publishes filtered LIDAR data to '/filtered_scan' topic.
            
        """
        super().__init__('lidar_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            '/filtered_scan',
            10)
        self.subscription  # prevent unused variable warning
    
        self.cut_off_angle = [
            [39, 50],       # Front-Left default:42-47
            [129, 140],     # Back-Left default:132-137
            [-141, -128],   # Back-Right default:-137--132 (Converted from 223-228)
            [-51, -38]      # Front-Right default:-47--42 (Converted from 313-318)
        ]

        self.counter = 0
        # self.timer = self.create_timer(0.5,self.terminal_callback)

        self.color = COLOR()
        self.get_logger().info(self.color.GREEN + "Lidar Filter Node Started." + self.color.END)

    def scan_callback(self, msg):
        """
            Callback function for incoming LIDAR data.
            Filters out readings in specified angular ranges by setting them to infinity.
        """
        filtered_msg = msg
        new_ranges = list(msg.ranges)
        
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i in range(len(new_ranges)):
            
            angle_rad = angle_min + (i * angle_inc) # 1. Get angle in RADIANS (Standard ROS ranges from -3.14 to +3.14)
            
            angle_deg = angle_rad * (180.0 / math.pi) # 2. Convert to DEGREES (-180 to 180)

            # 3. Check against cutoff list
            # We NO LONGER need to add 360. We use the raw negative values.
            for start_deg, end_deg in self.cut_off_angle:
                
                if start_deg <= angle_deg <= end_deg: # Check if angle falls between start and end
                    new_ranges[i] = float('inf')
                    break

        filtered_msg.ranges = new_ranges
        self.publisher.publish(filtered_msg)
        ################################################
        # self.debug(msg)
        
    def debug(self, msg):
        """
            Debug function to print all LIDAR data points.
        """
        for j in range(len(msg.ranges)):
            self.get_logger().info(f"Data {j} : {msg.ranges[j]}")
        self.get_logger().info("--------------------------------------------")

    def terminal_callback(self):
        """
            Timer callback to indicate that the Lidar Filter node is running.
        """
        GREEN = "\033[92m"
        STOP = "\033[0m"
        num_dots = (self.counter % 4)
        loading = ['\u2500', '\\', '|', '/']
        msg = f"\r{GREEN}[STATUS] Lidar Filter is running:{STOP} {loading[num_dots]}\033[K"
        print(msg, end='', flush=True)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    lidar_filter = LidarFilter()
    rclpy.spin(lidar_filter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()