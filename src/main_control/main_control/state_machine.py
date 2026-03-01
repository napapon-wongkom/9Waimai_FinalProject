import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.Idle_State = 1
        self.Navigation_State = 0
        self.Interaction_State = 0
        self.Detection_State = 0
        self.Allocation_State = 0
        self.Approach_State = 0
        self.Exit_State = 0
        


        self.state_publisher = self.create_publisher(
            Int32MultiArray,
            'state_machine',
            10
        )

        self.color = COLOR()
        self.timer = self.create_timer(0.01, self.state_handle)
        self.get_logger().info(self.color.GREEN + "State Machine Node Started." + self.color.END)

    def state_handle(self):
        state_msg = Int32MultiArray()
        
        
        
        
        
        
        
        state_msg.data = [
            self.Idle_State, # Idle_State
            0, # Navigation_State
            0, # Interaction_State
            0, # Detection_State
            0, # Allocation_State
            0, # Approach_State
            0, # Exit_State
            ]

        self.state_publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()