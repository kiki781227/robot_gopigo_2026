import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


# ─── PARAMÈTRES ──────────────────────────────────────────────────────────────

COLOR_RANGES = {
    'bleu': (
        np.array([100, 100, 50]),
        np.array([130, 255, 255])
    ),
    'vert': (
        np.array([40, 60, 50]),
        np.array([80, 255, 255])
    ),
    'rouge': (
        np.array([0, 120, 70]),
        np.array([10, 255, 255])
    ),
    'jaune': (
        np.array([20, 100, 100]),
        np.array([35, 255, 255])
    ),
}

MIN_AREA = 400
CROP_RATIO = 0.40   # 🔥 IMPORTANT (comme calibration)


# ─── NODE ────────────────────────────────────────────────────────────────────

class CubeDetectorNode(Node):

    def __init__(self):
        super().__init__('cube_detector_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pub_position = self.create_publisher(Point, '/cube/position', 10)
        self.pub_color    = self.create_publisher(String, '/cube/color', 10)

        self.get_logger().info(
            'Cube detector OPTIMISE - couleurs: ' + str(list(COLOR_RANGES.keys()))
        )

    # ─── CALLBACK IMAGE ───────────────────────────────────────────────────────
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ── CROP (🔥 GROS IMPACT)
        h, w = frame.shape[:2]
        crop_start = int(h * CROP_RATIO)
        frame = frame[crop_start:, :]

        # ── Correction couleur (comme calibration)
        b, g, r = cv2.split(frame)
        g = cv2.subtract(g, 10)
        frame = cv2.merge([b, g, r])

        # ── Filtrage + HSV
        bilateral = cv2.bilateralFilter(frame, 5, 10, 75)
        hsv = cv2.cvtColor(bilateral, cv2.COLOR_BGR2HSV)

        height, width = frame.shape[:2]
        center_x = width / 2
        center_y = height / 2

        best_cube  = None
        best_area  = 0
        best_color = None

        # ── Détection par couleur
        for color_name, (lower, upper) in COLOR_RANGES.items():

            mask = cv2.inRange(hsv, lower, upper)

            # 🔥 DOUBLE MASQUE ROUGE (CRUCIAL)
            if color_name == 'rouge':
                lower2 = np.array([170, lower[1], lower[2]])
                upper2 = np.array([179, upper[1], upper[2]])
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)

            # ── Nettoyage
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # ── Contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                if area < MIN_AREA:
                    continue

                if area > best_area:
                    best_area  = area
                    best_cube  = contour
                    best_color = color_name

        # ── Publication
        if best_cube is not None:
            M = cv2.moments(best_cube)

            if M['m00'] == 0:
                return

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            rel_x = cx - center_x
            rel_y = cy - center_y

            # Position
            pos_msg = Point()
            pos_msg.x = float(rel_x)
            pos_msg.y = float(rel_y)
            pos_msg.z = float(best_area)

            self.pub_position.publish(pos_msg)

            # Couleur
            color_msg = String()
            color_msg.data = best_color
            self.pub_color.publish(color_msg)

            self.get_logger().info(
                f'Cube {best_color} | pos=({rel_x:.0f},{rel_y:.0f}) | aire={best_area:.0f}'
            )

        else:
            self.get_logger().debug('Aucun cube detecte')


# ─── MAIN ────────────────────────────────────────────────────────────────────

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
