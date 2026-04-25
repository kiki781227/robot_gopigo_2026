"""
aruco_detector_node.py
──────────────────────
Détection ArUco avec calcul de distance et bearing (angle horizontal).

Fixes par rapport à l'ancienne version :
  - API OpenCV moderne (ArucoDetector)
  - Calcul distance via modèle pinhole
  - Calcul bearing (angle horizontal robot → marqueur)
  - Taille via sqrt(aire) au lieu de moyenne width/height

Stratégie multi-marqueurs : si plusieurs ArUco visibles dans une frame,
on publie le PLUS GROS (le plus proche). Le mission_node décidera si c'est
le bon ID ou s'il faut continuer à chercher.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from robot_gopigo_msgs.msg import ArucoDetect


def polygon_area(corners):
    """Aire d'un quadrilatère via formule de Shoelace."""
    x = corners[:, 0]
    y = corners[:, 1]
    return 0.5 * abs(
        x[0]*y[1] - x[1]*y[0] +
        x[1]*y[2] - x[2]*y[1] +
        x[2]*y[3] - x[3]*y[2] +
        x[3]*y[0] - x[0]*y[3]
    )


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('marker_size_m', 0.054)   # 5.4 cm
        self.declare_parameter('focal_length_px', 730.0) # focale caméra (px)
        # À calibrer empiriquement : placer ArUco à 1m, mesurer la taille px
        # puis focal = taille_px * 1m / marker_size_m

        self.declare_parameter('dictionary', 'DICT_4X4_50')

        self.marker_size_m   = self.get_parameter('marker_size_m').value
        self.focal_length_px = self.get_parameter('focal_length_px').value
        dict_name            = self.get_parameter('dictionary').value

        # ── Bridge + OpenCV ─────────────────────────────────────────────────
        self.bridge = CvBridge()

        # Mapping dictionnaires
        dict_map = {
            'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
        }
        if dict_name not in dict_map:
            self.get_logger().error(f"Dictionnaire inconnu : {dict_name}")
            raise ValueError(f"Unknown dict: {dict_name}")

        # Compatibilité API : OpenCV 4.7+ (nouvelle) vs 4.6 et antérieur (legacy)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_map[dict_name])

        if hasattr(cv2.aruco, 'ArucoDetector'):
            # API moderne (OpenCV 4.7+)
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.detector_params
            )
            self._use_modern_api = True
        else:
            # API legacy (OpenCV < 4.7)
            if hasattr(cv2.aruco, 'DetectorParameters_create'):
                self.detector_params = cv2.aruco.DetectorParameters_create()
            else:
                self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = None
            self._use_modern_api = False

        self.get_logger().info(
            f"API ArUco : {'moderne (4.7+)' if self._use_modern_api else 'legacy (<4.7)'}"
        )

        # ── ROS interfaces ──────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.pub_aruco = self.create_publisher(ArucoDetect, '/aruco_detect', 10)

        # Log périodique
        self.frame_count = 0

        self.get_logger().info(
            f"Aruco detector prêt ─ dict={dict_name}, "
            f"marker_size={self.marker_size_m*100:.1f}cm, "
            f"focal={self.focal_length_px}px"
        )

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Détection compatible double API
        if self._use_modern_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

        aruco_msg = ArucoDetect()
        aruco_msg.img_width = width
        aruco_msg.img_height = height

        if ids is not None and len(ids) > 0:
            # S'il y a plusieurs marqueurs, on garde le plus gros (le plus proche)
            best_idx = 0
            best_size = 0.0
            best_corners = None

            for i in range(len(ids)):
                c = corners[i][0]  # shape (4, 2)
                size_px = math.sqrt(polygon_area(c))
                if size_px > best_size:
                    best_size = size_px
                    best_idx = i
                    best_corners = c

            marker_id = int(ids[best_idx][0])
            c = best_corners

            # Centre du marqueur (en pixels)
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())

            # Distance via formule pinhole
            # size_px = (marker_size_m * focal) / distance
            # → distance = (marker_size_m * focal) / size_px
            if best_size > 0:
                distance_m = (self.marker_size_m * self.focal_length_px) / best_size
            else:
                distance_m = 0.0

            # Bearing : angle horizontal depuis l'axe optique
            # bearing > 0 : marqueur à droite
            # bearing < 0 : marqueur à gauche
            image_center_x = width / 2.0
            bearing_rad = math.atan2(cx - image_center_x, self.focal_length_px)

            aruco_msg.visible      = True
            aruco_msg.marker_id    = marker_id
            aruco_msg.cx           = cx
            aruco_msg.cy           = cy
            aruco_msg.size         = float(best_size)
            aruco_msg.distance_m   = float(distance_m)
            aruco_msg.bearing_rad  = float(bearing_rad)

            # Log toutes les 15 frames (≈2 fois par sec à 30fps)
            if self.frame_count % 15 == 0:
                self.get_logger().info(
                    f"ArUco id={marker_id} | "
                    f"px=({cx},{cy}) size={best_size:.0f}px | "
                    f"dist={distance_m:.2f}m | "
                    f"bearing={math.degrees(bearing_rad):+.1f}°"
                )

        else:
            aruco_msg.visible     = False
            aruco_msg.marker_id   = -1
            aruco_msg.cx          = 0
            aruco_msg.cy          = 0
            aruco_msg.size        = 0.0
            aruco_msg.distance_m  = 0.0
            aruco_msg.bearing_rad = 0.0

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
