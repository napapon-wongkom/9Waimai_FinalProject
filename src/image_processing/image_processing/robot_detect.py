import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class COLOR():
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    END = '\033[0m'

class ArmDetect(Node):
    def __init__(self):
        super().__init__('robot_image_processing')

        self.model = YOLO('src/image_processing/image_processing/elevator_best_ncnn_model', task='detect')

        # for debugging
        self.video_pub = self.create_publisher(
            CompressedImage,
            'video_frames/compressed',
            10
        )

        self.arm_command_pub = self.create_publisher(
            Int32MultiArray,
            'arm_command',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.capture_handle
        )

        self.color = COLOR()
        
        self.target_class = None

        self.up_arm_cmd = 0
        self.down_arm_cmd = 0

        self.error_threshold = 20

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 512)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)

        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 225)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 1000)

        self.br = CvBridge()

        self.get_logger().info(self.color.GREEN + "Arm Image Processing Node Started." + self.color.END)

    def capture_handle(self):
        arm_cmd = Int32MultiArray()
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (512, 512))

        center_img = (256, 256)

        ########################################
        self.target_class = 0

        if ret:
            # process the frame
            results = self.model.predict(
                source=frame,
                save=False,
                conf=0.6,
                verbose=False
            )

            annotated_frame = results[0].plot()
            cv2.drawMarker(annotated_frame, center_img, (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    if class_id == self.target_class:
                        x, y, w, h = box.xywh[0].tolist()

                        target_center = (int(x), int(y))

                        cv2.arrowedLine(
                            annotated_frame,
                            center_img,
                            target_center,
                            (0, 0, 255),
                            2,
                            tipLength = 0.2
                        )


                        err_x = x - 256
                        err_y = -(y - 256)
                        

                        if abs(err_x) > self.error_threshold:
                            # Set down_arm_cmd to 1 or -1 based on error direction
                            self.down_arm_cmd = 1 if err_x > 0 else -1
                        elif abs(err_y) > self.error_threshold:
                            # Only checks Y if X is already "good"
                            self.up_arm_cmd = 1 if err_y > 0 else -1


                        self.get_logger().info(f"Moving arm: x={err_x}, y={err_y}")

            msg = self.br.cv2_to_compressed_imgmsg(annotated_frame, dst_format="jpg")
            self.video_pub.publish(msg)
        else:
            self.get_logger().warn("Failed to capture video frame.")

        arm_cmd.data = [self.down_arm_cmd, self.up_arm_cmd]
        self.arm_command_pub.publish(arm_cmd)

        
    
def main(args=None):
    rclpy.init(args=args)
    node = ArmDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()