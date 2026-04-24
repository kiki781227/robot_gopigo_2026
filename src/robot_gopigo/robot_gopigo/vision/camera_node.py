"""
camera_node.py
──────────────
Capture d'images via OpenCV VideoCapture avec paramètres optimisés
pour fixer le rendu verdâtre et améliorer les performances.

Fixes :
  - Format MJPG (plus rapide que YUYV par défaut)
  - Balance des blancs manuelle (plus de dérive verdâtre)
  - Buffer réduit à 1 (latence minimale)
  - Ouverture via V4L2 explicite (plus fiable sur Pi/Ubuntu)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',    30)
        self.declare_parameter('device_index', 0)

        # Balance des blancs manuelle
        # 3000K = chaud / jaune (incandescent)
        # 4500K = neutre intérieur (néons blancs)
        # 6500K = froid / bleu (lumière du jour)
        self.declare_parameter('wb_temperature', 4500)
        self.declare_parameter('auto_wb',        False)

        # Exposition (auto par défaut, fixe si besoin)
        self.declare_parameter('auto_exposure',  True)
        self.declare_parameter('exposure_value', 156)  # utilisé si auto_exposure=False

        self.width         = self.get_parameter('width').value
        self.height        = self.get_parameter('height').value
        self.fps           = self.get_parameter('fps').value
        device_index       = self.get_parameter('device_index').value
        wb_temp            = self.get_parameter('wb_temperature').value
        auto_wb            = self.get_parameter('auto_wb').value
        auto_exposure      = self.get_parameter('auto_exposure').value
        exposure_value     = self.get_parameter('exposure_value').value

        # ── Publisher + Bridge ──────────────────────────────────────────────
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # ── Ouverture caméra via V4L2 ───────────────────────────────────────
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error("Impossible d'ouvrir la caméra !")
            raise RuntimeError("Camera not available")

        # Format de pixel : MJPG (beaucoup plus rapide que YUYV par défaut)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Résolution et FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)

        # Buffer réduit (latence minimale)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Balance des blancs : désactiver l'auto qui part dans les verts
        if not auto_wb:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp)
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)

        # Exposition
        if not auto_exposure:
            # Mode manuel (valeur V4L2 : 1 = manuel, 3 = auto)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

        # Vérification des paramètres effectifs
        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))

        self.get_logger().info(
            f"Caméra ouverte : {actual_w}x{actual_h} @ {actual_fps}fps | "
            f"WB={'auto' if auto_wb else f'{wb_temp}K'} | "
            f"Exposure={'auto' if auto_exposure else f'{exposure_value}'}"
        )

        # ── Timer capture ───────────────────────────────────────────────────
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)

        # Compteur pour log périodique
        self.frame_count = 0

    def capture_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Frame non capturée')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher.publish(msg)

            self.frame_count += 1
            # Log toutes les 150 frames (≈ 5s à 30 fps)
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
