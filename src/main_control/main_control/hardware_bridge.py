import rclpy
from std_msgs.msg import Float32MultiArray, String
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
import math
import tf_transformations
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Robot Physical Parameters
        self.wheel_radius = 0.1016 # 8-Inch wheel 
        self.wheel_base = 0.475
        self.ticks_per_rev = 600.0 * 2

        # State Variable
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.first_run = True

        self.encode_tick_left = 0
        self.encode_tick_right = 0
        self.yaw = 0

        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'physical_data',
            self.data_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry, 
            'odom', 
            10
            )

        self.odom_debug = self.create_publisher(
            String, 
            'odom_debug', 
            10
            )

        self.cmd_send = self.create_publisher(
            Float32MultiArray,
            'cmd_send',
            10
            )
        
        self.timer = self.create_timer(0.05, self.update_odom) # 20Hz

        
        self.color = COLOR()
        self.get_logger().info(self.color.GREEN + "Hardware Bridge Node Started." + self.color.END)
    
    def data_callback(self, msg):
        incoming_data = msg.data

        physical_datas = Float32MultiArray()
        self.encode_tick_left = incoming_data[0]
        self.encode_tick_right = incoming_data[1]
        self.yaw = incoming_data[2]

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        w_z = msg.angular.z

        # Inverse kinematics
        v_left = v_x - (w_z * self.wheel_base / 2.0)
        v_right = v_x + (w_z * self.wheel_base / 2.0)

        #Prepare data for ESP32
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [v_left, v_right]

        self.cmd_send.publish(cmd_msg)

    def update_odom(self):
        if self.first_run:
            self.prev_left, self.prev_right = self.encode_tick_left, self.encode_tick_right
            self.first_run = False
            return
        # Handle encoder different
        delta_left = (self.encode_tick_left - self.prev_left) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        delta_right = (self.encode_tick_right - self.prev_right) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        self.prev_left, self.prev_right = self.encode_tick_left, self.encode_tick_right

        self.left_wheel_angle = (self.encode_tick_left / self.ticks_per_rev) * (2 * math.pi)
        self.right_wheel_angle = (self.encode_tick_right / self.ticks_per_rev) * (2 * math.pi)

        # Handle encoder to measure direction
        delta_th_encoder = (delta_right - delta_left) / self.wheel_base
        encoder_yaw_prediction = self.th + delta_th_encoder

        # Calculate dofference and normalize to [-pi, pi]
        diff = self.yaw - encoder_yaw_prediction
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        # Apply Fusion
        alpha = 0.95
        self.th = encoder_yaw_prediction + (1.0 - alpha) * diff
        
        
        
        delta_center = (delta_left + delta_right) / 2.0
        self.x += delta_center * math.cos(self.th)
        self.y += delta_center * math.sin(self.th)

        self.publish_odom()

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # Boardcast the TF [Make robot move in RViz]
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publish Joint State [Move the wheel out of center]
        js = JointState()
        js.header.stamp = now
        js.name = [
            'left_drive_wheel_joint', 
            'right_drive_wheel_joint',
            'caster_wheel_joint_fl',
            'caster_wheel_joint_fr',
            'caster_wheel_joint_rl',
            'caster_wheel_joint_rr'
            ]
        js.position = [
            self.left_wheel_angle, 
            self.right_wheel_angle,
            0.0,
            0.0,
            0.0,
            0.0
            ]
        self.joint_pub.publish(js)


        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odomdebug = String()
        odomdebug.data = f"X: {self.x} ,Y: {self.y} ,Theta: {self.th}"

        self.odom_debug.publish(odomdebug)
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()