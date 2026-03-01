import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class FloorManager(Node):
    def __init__(self):
        super().__init__('floor_manager')

        self.color = COLOR()

        self.current_floor = 1

        # Path to map
        self.floor_data = {
            1: {'path': '~/finalproject_ws/src/main_system/maps/floor_1.yaml', 'x': 0.0, 'y': 0.0},
            2: {'path': '~/finalproject_ws/src/main_system/maps/floor_2.yaml', 'x': 0.0, 'y': 0.0},
            3: {'path': '~/finalproject_ws/src/main_system/maps/floor_3.yaml', 'x': 0.0, 'y': 0.0}
        }

        self.level_subscription(
            Float32MultiArray, 
            'level_data',
            self.altitude_callback,
            10
            )

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info(self.color.GREEN + "Floor Manager Node Started." + self.color.END)

    def switch_floor(self, floor_number):
        if floor_number not in self.floor_data:
            self.get_logger().error(f"Floor {floor_number} not found in library!")
            return

        request = LoadMap.Request()
        request.map_url = self.floor_data[floor_number]['path']

        self.get_logger().info(self.color.BLUE + f"Switching to Floor {floor_number}..." + self.color.END)
        future = self.map_client.call_async(request)
        future.add_done_callback(lambda f: self.map_load_callback(f, floor_number))

    def map_load_callback(self, future, floor_number):
        try:
            response = future.result()
            if response.result == 0: # -> SUCCESS
                self.get_logger().info(f"Successfully loaded map for Floor {floor_number}")
                self.set_initial_pose(
                    self.floor_data[floor_number]['x'],
                    self.floor_data[floor_number]['y']
                ) # Relocalize the robot at the elevator door
            else:
                self.get_logger().error("Map server failed to load map.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def set_initial_pose(self, x_pos, y_pos):
        """
            Tells AMCL exactly where the robot is standing after the floor swap.
            You need to find these coordinates for your specific elevator door.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x_pos
        msg.pose.pose.position.y = y_pos
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance = [1.0] * 36

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(self.color.CYAN + "Initial pose sent to AMCL." + self.color.END)

    def altitude_callback(self, msg):
        altidude = msg.data[3]
        if 3.0 < altitude < 4.0 and self.current_floor != 2:
            self.switch_floor(2)
            self.current_floor = 2

def main():
    rclpy.init()
    node = FloorManager()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()      