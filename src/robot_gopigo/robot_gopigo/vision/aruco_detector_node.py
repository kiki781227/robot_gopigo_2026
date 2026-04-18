import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from robot_gopigo_msgs.msg import ArucoDetect
 
 
class ArucoDetectorNode(Node):
 
    def __init__(self):
        super().__init__('aruco_detector_node')
 
        self.bridge = CvBridge()
 
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
 
        self.pub_aruco = self.create_publisher(ArucoDetect, '/aruco_detect', 10)
 
        # API compatible OpenCV < 4.7
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters_create()  
 
        self.get_logger().info('Aruco detector pret - topic /aruco_detect')
 
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
 
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )
 
        aruco_msg = ArucoDetect()
        aruco_msg.img_width = width
        aruco_msg.img_height = height
 
        if ids is not None and len(ids) > 0:
            marker_id = int(ids[0][0])
            marker_corners = corners[0][0]
 
            cx = int(marker_corners[:, 0].mean())
            cy = int(marker_corners[:, 1].mean())
 
            width_px = ((marker_corners[0][0] - marker_corners[1][0]) ** 2 +
                        (marker_corners[0][1] - marker_corners[1][1]) ** 2) ** 0.5
            height_px = ((marker_corners[1][0] - marker_corners[2][0]) ** 2 +
                         (marker_corners[1][1] - marker_corners[2][1]) ** 2) ** 0.5
            size = float((width_px + height_px) / 2.0)
 
            aruco_msg.visible = True
            aruco_msg.marker_id = marker_id
            aruco_msg.cx = cx
            aruco_msg.cy = cy
            aruco_msg.size = size
 
            self.get_logger().info(
                f'Aruco id={marker_id} | cx={cx} cy={cy} | size={size:.1f}'
            )
 
        else:
            aruco_msg.visible = False
            aruco_msg.marker_id = -1
            aruco_msg.cx = 0
            aruco_msg.cy = 0
            aruco_msg.size = 0.0
 
        self.pub_aruco.publish(aruco_msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()