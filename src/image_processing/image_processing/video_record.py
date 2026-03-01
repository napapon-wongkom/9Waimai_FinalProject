import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class Recorder(Node):
    def __init__(self):
        super().__init__('record')

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.cam_callback,
            10
        )

        self.bridge = CvBridge()
        self.video_writer = None
        self.output_file = 'elevator_data_2.avi'
        self.fps = 20.0

        self.prev_time = 0.0
        self.curr_time = 0.0

        self.get_logger().info('Video Recorder Node has started. Waiting for frame...')

    def cam_callback(self, data):
        try:
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # self.crosshair(current_frame)
            # self.fps_debug(current_frame)


            if self.video_writer is None:
                height, width, _ = current_frame.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(
                    self.output_file, fourcc, self.fps, (width, height)
                )
                self.get_logger().info(f"Recording Started: {width}x{height} at {self.fps} FPS")
            
            self.video_writer.write(current_frame)
            # cv2.imshow("Recording...", current_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()

    def fps_debug(self, frame):
        self.curr_time = time.time()
        if self.prev_time != 0:
            time_diff = self.curr_time - self.prev_time

            fps = 1 / time_diff
            # self.get_logger().info(f"FPS: {fps}")
            cv2.putText(
                frame,
                f"FPS: {int(fps)}",
                (10,50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
        
        self.prev_time = self.curr_time

    def crosshair(self, frame):
        height, width, _ = frame.shape 
        # shape = (480, 640, 3)

        center_x = int(width / 2)
        center_y = int(height / 2)

        line_length_x = width
        line_length_y = height
        thickness = 2
        color = (0, 0, 255)
        
        cv2.line( # Horizon line
            frame,
            (0, center_y),
            (width, center_y),
            color,
            thickness
        )

        cv2.line( # Verticle line
            frame,
            (center_x, 0),
            (center_x, height),
            color,
            thickness
        )

        # self.get_logger().info(f"Center : {center_x},{center_y} (x,y)")
        return center_x, center_y

def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except Keyboardinterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()