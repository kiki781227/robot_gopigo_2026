import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # Paramètres configurables
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self.width  = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps    = self.get_parameter('fps').value

        # Publisher — publie les images brutes
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Bridge OpenCV <-> ROS 2
        self.bridge = CvBridge()

        # Ouvre la caméra
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)

        if not self.cap.isOpened():
            self.get_logger().error('Impossible d\'ouvrir la camera !')
            return

        self.get_logger().info(
            f'Camera ouverte : {self.width}x{self.height} @ {self.fps}fps'
        )

        # Timer — capture une image à chaque intervalle
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)

    def capture_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Frame non capturee, on passe...')
            return

        # Convertit en message ROS 2 et publie
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
