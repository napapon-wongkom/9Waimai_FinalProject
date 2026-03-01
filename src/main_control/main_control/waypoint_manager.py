import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class Waypoint_manager(Node):
    def __init__(self):
        super().__init__('waypoint_management_node')
        
        self.color = COLOR()
        self.nav = BasicNavigator()

        self.target_sub = self.create_subscriber(
            Int32,
            'target_floor',
            self.target_callback,
            10
        )

        self.cur_floor_sub = self.create_subscriber(
            Int32,
            'current_floor',
            self.current_floor_callback,
            10
        )

        self.state_sub = self.create_subscriber(
            String,
            'robot_state',
            self.state_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            'robot_state',
            10
        )

        self.timer = self.create_timer(1.0, self.control_loop)

        self.target_floor = 0
        self.robot_state = 'IDLE'
        self.waypoints = self.load_waypoints('src/main_system/config/waypoints.yaml')

        self.current_floor = 1
        target_point = "home_point_1"
        self.execute_sequence()

    def target_callback(self, msg):
        self.target_floor = msg.data
        if self.target_floor != 0:
            self.robot_state = 'GOTOGOAL'

            state_msg = String()
            state_msg.data = self.robot_state
            self.state_pub.publish(state_msg)

            self.get_logger().info(self.color.CYAN + f"TARGET RECEIVE: FLOOR {self.target_floor}" + self.color.END)
  
    def state_callback(self, msg):
        self.robot_state = msg.data

    def current_floor_callback(self, msg):
        self.current_floor = msg.data


    def control_loop(self):
        robot_state_pub = String()
        if self.robot_state == 'GOTOGOAL':
            self.get_logger().info(self.color.CYAN + "START SEQUENCE!!!" + self.color.END)
            if self.current_floor == 1 and self.target_floor == 1:
                goal = self.waypoints['floor_1']['home_point_1']
                goal_pose = self.create_pose(goal)

                self.nav.goToPose(goal_pose)

                self.robot_state = 'NAVIGATE'

        elif self.robot_state == 'NAVIGATE':
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(self.color.GREEN + "Goal Reached!!!" + self.color.END)
                    self.robot_state = 'IDLE'
                else:
                    self.get_logger().error("Navigation failed!")
                    self.current_state = 'IDLE'

            



    def load_waypoints(self, file_path):
        try:
            with open(file_path, 'r') as f:
                return yaml.safe_load(f)['points']
        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} not found!")
            return None

    def create_pose(self, data):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = data['x']
        pose.pose.position.y = data['y']
        pose.pose.orientation.z = data['z_orient']
        pose.pose.orientation.w = data['w_orient']
        return pose

    def execute_sequence(self):
        self.get_logger().info(self.color.CYAN + "START SEQUENCE!!!" + self.color.END)
        
        self.get_logger().info(self.color.YELLOW + "Moving to elevaor" + self.color.END)
        goal = self.create_pose(self.waypoints['floor_1']['elevator1'])
        self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            pass

        self.get_logger().info(self.color.GREEN + "FINISH SEQUENCE" + self.color.END)

def main(args=None):
    rclpy.init(args=args)
    node = Waypoint_manager()
    rclpy.shutdown()

if __name__ == '__main__':
    main()