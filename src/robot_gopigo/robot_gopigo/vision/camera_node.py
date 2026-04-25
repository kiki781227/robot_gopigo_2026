"""
camera_node.py
──────────────
Capture caméra avec :
  - V4L2 + MJPG (perf)
  - WB caméra manuelle
  - Exposure réglable (auto ou manuelle)
  - Gray World logicielle (anti-verdâtre)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def gray_world_correction(frame, strength=1.0):
    """Balance des blancs Gray World (atténue dominante verte)."""
    f = frame.astype(np.float32)
    b, g, r = cv2.split(f)

    avg_b = np.mean(b)
    avg_g = np.mean(g)
    avg_r = np.mean(r)
    avg_gray = (avg_b + avg_g + avg_r) / 3.0

    if avg_b > 0 and avg_g > 0 and avg_r > 0:
        gb = avg_gray / avg_b
        gg = avg_gray / avg_g
        gr = avg_gray / avg_r
    else:
        gb = gg = gr = 1.0

    gb = 1.0 + (gb - 1.0) * strength
    gg = 1.0 + (gg - 1.0) * strength
    gr = 1.0 + (gr - 1.0) * strength

    b = np.clip(b * gb, 0, 255)
    g = np.clip(g * gg, 0, 255)
    r = np.clip(r * gr, 0, 255)

    return cv2.merge([b, g, r]).astype(np.uint8)


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',    30)
        self.declare_parameter('device_index', 0)

        self.declare_parameter('wb_temperature', 4500)
        self.declare_parameter('auto_wb',        False)

        self.declare_parameter('auto_exposure',  False)
        self.declare_parameter('exposure_value', 500)

        self.declare_parameter('software_wb_enabled',  True)
        self.declare_parameter('software_wb_strength', 0.7)

        # ── Lecture ─────────────────────────────────────────────────────────
        self.width         = self.get_parameter('width').value
        self.height        = self.get_parameter('height').value
        self.fps           = self.get_parameter('fps').value
        device_index       = self.get_parameter('device_index').value
        wb_temp            = self.get_parameter('wb_temperature').value
        auto_wb            = self.get_parameter('auto_wb').value
        auto_exposure      = self.get_parameter('auto_exposure').value
        exposure_value     = self.get_parameter('exposure_value').value

        self.sw_wb_enabled  = self.get_parameter('software_wb_enabled').value
        self.sw_wb_strength = self.get_parameter('software_wb_strength').value

        # ── Publisher ───────────────────────────────────────────────────────
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # ── Caméra ──────────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Impossible d'ouvrir la caméra !")
            raise RuntimeError("Camera not available")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        if not auto_wb:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp)
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)

        if not auto_exposure:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)

        self.get_logger().info(
            f"Caméra ouverte : {actual_w}x{actual_h} | "
            f"WB={'auto' if auto_wb else f'{wb_temp}K'} | "
            f"Exposure: {actual_exposure} ({'auto' if auto_exposure else 'manuel'}) | "
            f"WB-SW={'on (s=%.1f)' % self.sw_wb_strength if self.sw_wb_enabled else 'off'}"
        )

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)
        self.frame_count = 0

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame non capturée')
            return

        if self.sw_wb_enabled:
            frame = gray_world_correction(frame, strength=self.sw_wb_strength)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher.publish(msg)

            self.frame_count += 1
            if self.frame_count % 150 == 0:
                self.get_logger().info(f"Frames publiées : {self.frame_count}")

        except Exception as e:
            self.get_logger().error(f"Erreur publication frame : {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
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
