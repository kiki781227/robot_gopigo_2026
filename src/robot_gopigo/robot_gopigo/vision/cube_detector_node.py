"""
cube_detector_node.py
─────────────────────
Détection de cubes colorés version "exposure géré" (avant le retour à l'original).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from robot_gopigo_msgs.msg import CubeDetect, ArucoDetect


class CubeDetectorNode(Node):

    def __init__(self):
        super().__init__('cube_detector_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('hsv_blue',   [100, 100, 50,  130, 255, 255])
        self.declare_parameter('hsv_green',  [40,  60,  50,  80,  255, 255])
        self.declare_parameter('hsv_red1',   [0,   120, 70,  10,  255, 255])
        self.declare_parameter('hsv_red2',   [170, 120, 70,  179, 255, 255])
        self.declare_parameter('hsv_yellow', [20,  100, 100, 35,  255, 255])

        self.declare_parameter('min_area', 400)
        self.declare_parameter('max_area', 40000)

        self.declare_parameter('crop_top_ratio', 0.5)
        self.declare_parameter('aruco_exclusion_radius_px', 120)
        self.declare_parameter('aruco_filter_enabled', True)

        self.declare_parameter('distance_K', 5.0)
        self.declare_parameter('log_every_n_frames', 30)

        # ── Lecture ─────────────────────────────────────────────────────────
        self.color_ranges = self._load_color_ranges()
        self.min_area               = self.get_parameter('min_area').value
        self.max_area               = self.get_parameter('max_area').value
        self.crop_top_ratio         = self.get_parameter('crop_top_ratio').value
        self.aruco_exclusion_radius = self.get_parameter('aruco_exclusion_radius_px').value
        self.aruco_filter_enabled   = self.get_parameter('aruco_filter_enabled').value
        self.distance_K             = self.get_parameter('distance_K').value
        self.log_every_n            = self.get_parameter('log_every_n_frames').value

        # ── ROS ─────────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.last_aruco = None
        self.create_subscription(ArucoDetect, '/aruco_detect', self._aruco_cb, 10)

        self.pub_cube = self.create_publisher(CubeDetect, '/cube_detect', 10)

        self.frame_count = 0

        self.get_logger().info(
            f"Cube detector prêt ─ couleurs: {list(self.color_ranges.keys())} | "
            f"min_area={self.min_area}, crop={self.crop_top_ratio*100:.0f}% | "
            f"aruco_filter={'on' if self.aruco_filter_enabled else 'off'}"
        )

    def _load_color_ranges(self):
        def get(name):
            v = self.get_parameter(name).value
            return (np.array(v[:3], dtype=np.uint8),
                    np.array(v[3:], dtype=np.uint8))
        return {
            'bleu':  get('hsv_blue'),
            'vert':  get('hsv_green'),
            'jaune': get('hsv_yellow'),
            'rouge': (get('hsv_red1'), get('hsv_red2')),
        }

    def _aruco_cb(self, msg):
        self.last_aruco = msg

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        full_h, full_w = frame.shape[:2]

        # Crop vertical
        crop_start = int(full_h * self.crop_top_ratio)
        cropped = frame[crop_start:, :]

        # Bilateral
        bilateral = cv2.bilateralFilter(cropped, 5, 50, 50)
        hsv = cv2.cvtColor(bilateral, cv2.COLOR_BGR2HSV)

        best_cube = None
        best_area = 0.0
        best_color = None
        best_cx_full = 0
        best_cy_full = 0

        for color_name, ranges in self.color_ranges.items():

            if color_name == 'rouge':
                (l1, u1), (l2, u2) = ranges
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, l1, u1),
                    cv2.inRange(hsv, l2, u2)
                )
            else:
                lower, upper = ranges
                mask = cv2.inRange(hsv, lower, upper)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area or area > self.max_area:
                    continue

                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue

                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                cx_full = cx
                cy_full = cy + crop_start

                if self.aruco_filter_enabled and self._is_near_visible_aruco(cx_full, cy_full):
                    continue

                if area > best_area:
                    best_area = float(area)
                    best_cube = contour
                    best_color = color_name
                    best_cx_full = cx_full
                    best_cy_full = cy_full

        cube_msg = CubeDetect()
        cube_msg.img_width = full_w
        cube_msg.img_height = full_h

        if best_cube is not None:
            distance_est = self.distance_K / math.sqrt(best_area) if best_area > 0 else 0.0

            cube_msg.visible             = True
            cube_msg.color               = best_color
            cube_msg.cx                  = best_cx_full
            cube_msg.cy                  = best_cy_full
            cube_msg.area                = best_area
            cube_msg.distance_estimate_m = float(distance_est)

            if self.frame_count % self.log_every_n == 0:
                self.get_logger().info(
                    f"Cube {best_color} | px=({best_cx_full},{best_cy_full}) "
                    f"area={best_area:.0f} dist≈{distance_est:.2f}m"
                )
        else:
            cube_msg.visible             = False
            cube_msg.color               = ""
            cube_msg.cx                  = 0
            cube_msg.cy                  = 0
            cube_msg.area                = 0.0
            cube_msg.distance_estimate_m = 0.0

        self.pub_cube.publish(cube_msg)

    def _is_near_visible_aruco(self, cx, cy):
        if self.last_aruco is None or not self.last_aruco.visible:
            return False
        dx = cx - self.last_aruco.cx
        dy = cy - self.last_aruco.cy
        r = self.aruco_exclusion_radius
        return (dx*dx + dy*dy) < r*r


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
