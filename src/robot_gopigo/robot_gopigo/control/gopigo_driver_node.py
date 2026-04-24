"""
gopigo_driver_node.py
─────────────────────
Driver unifié pour le GoPiGo3 : fusion cmd_vel + odométrie + servo.
UNE SEULE instance gopigo3.GoPiGo3() pour éviter les conflits SPI et les
lenteurs de démarrage.

Topics :
  - Subscribe : /cmd_vel (geometry_msgs/Twist)       → commande moteurs
  - Subscribe : /servo_cmd (std_msgs/Float32)        → angle servo (0-180°)
  - Publish   : /odom (nav_msgs/Odometry)            → pose du robot
  - TF        : odom → base_link
"""

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros

import gopigo3   # API bas-niveau : moteurs, encodeurs, servo


class GoPiGoDriverNode(Node):

    def __init__(self):
        super().__init__('gopigo_driver_node')

        # ── Paramètres physiques (GoPiGo3) ──────────────────────────────────
        self.declare_parameter('wheel_base',        0.117)   # entraxe en mètres
        self.declare_parameter('wheel_radius',      0.03325) # rayon roue en m (66.5mm diamètre)
        self.declare_parameter('ticks_per_meter',   1655)    # calibration expérimentale
        self.declare_parameter('max_dps',           500)     # limite DPS moteurs
        self.declare_parameter('min_dps',           0)       # 0 = désactivé
        self.declare_parameter('cmd_timeout',       0.5)     # watchdog arrêt (s)
        self.declare_parameter('odom_rate',         20.0)    # fréquence odo (Hz)

        self.declare_parameter('invert_motors',     False)   # True si moteurs inversés
        self.declare_parameter('swap_motors',       False)   # True si L/R à swaper

        self.declare_parameter('servo_port',        1)       # 1 = SERVO_1, 2 = SERVO_2

        self.wheel_base      = self.get_parameter('wheel_base').value
        self.wheel_radius    = self.get_parameter('wheel_radius').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.max_dps         = self.get_parameter('max_dps').value
        self.min_dps         = self.get_parameter('min_dps').value
        self.cmd_timeout     = self.get_parameter('cmd_timeout').value
        self.odom_rate       = self.get_parameter('odom_rate').value
        self.invert_motors   = self.get_parameter('invert_motors').value
        self.swap_motors     = self.get_parameter('swap_motors').value
        servo_port_num       = self.get_parameter('servo_port').value

        self.meters_per_tick = 1.0 / self.ticks_per_meter
        self.RAD_TO_DPS      = 180.0 / math.pi

        # ── Initialisation du matériel (UNE SEULE instance) ─────────────────
        try:
            self.gpg = gopigo3.GoPiGo3()
            self.get_logger().info(
                f"GoPiGo3 initialisé ─ batterie: {self.gpg.get_voltage_battery():.2f} V"
            )
        except Exception as e:
            self.get_logger().error(f"Erreur init GoPiGo3: {e}")
            raise

        # Sélection du port servo
        if servo_port_num == 2:
            self.servo_port = self.gpg.SERVO_2
        else:
            self.servo_port = self.gpg.SERVO_1

        # ── Reset encodeurs à zéro ──────────────────────────────────────────
        left_raw  = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
        right_raw = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)
        self.gpg.offset_motor_encoder(self.gpg.MOTOR_LEFT,  left_raw)
        self.gpg.offset_motor_encoder(self.gpg.MOTOR_RIGHT, right_raw)
        self.left_prev  = 0
        self.right_prev = 0

        # ── État odométrie ──────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ── ROS interfaces ──────────────────────────────────────────────────
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_servo = self.create_subscription(
            Float32, '/servo_cmd', self.servo_callback, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer odométrie
        self.odom_dt = 1.0 / self.odom_rate
        self.odom_timer = self.create_timer(self.odom_dt, self.update_odometry)

        # Watchdog
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info("gopigo_driver_node prêt ─ en attente de commandes")

    # ══════════════════════════════════════════════════════════════════════
    # COMMANDE MOTEURS
    # ══════════════════════════════════════════════════════════════════════
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x    # m/s
        angular = msg.angular.z   # rad/s

        # Cinématique différentielle : vitesse de chaque roue (m/s)
        left_speed_mps  = linear - (angular * self.wheel_base / 2.0)
        right_speed_mps = linear + (angular * self.wheel_base / 2.0)

        # Conversion m/s → rad/s → DPS  (v = ω·r ⇒ ω = v/r)
        left_dps  = int((left_speed_mps  / self.wheel_radius) * self.RAD_TO_DPS)
        right_dps = int((right_speed_mps / self.wheel_radius) * self.RAD_TO_DPS)

        if self.invert_motors:
            left_dps  = -left_dps
            right_dps = -right_dps
        if self.swap_motors:
            left_dps, right_dps = right_dps, left_dps

        # Clamp max
        left_dps  = max(-self.max_dps, min(self.max_dps, left_dps))
        right_dps = max(-self.max_dps, min(self.max_dps, right_dps))

        # Seuil minimum (désactivé par défaut)
        if self.min_dps > 0:
            if 0 < abs(left_dps) < self.min_dps:
                left_dps = self.min_dps if left_dps > 0 else -self.min_dps
            if 0 < abs(right_dps) < self.min_dps:
                right_dps = self.min_dps if right_dps > 0 else -self.min_dps

        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  left_dps)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, right_dps)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  0)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 0)

    # ══════════════════════════════════════════════════════════════════════
    # SERVO
    # ══════════════════════════════════════════════════════════════════════
    def servo_callback(self, msg: Float32):
        """
        Pilote le servo via set_servo(port, pulse_width_us).
        Conversion angle → pulse :
          0°   → 500 µs
          90°  → 1500 µs
          180° → 2500 µs
        """
        try:
            angle = max(0.0, min(180.0, float(msg.data)))
            pulse_us = int(500 + (angle / 180.0) * 2000)
            self.gpg.set_servo(self.servo_port, pulse_us)
            self.get_logger().info(f"Servo → {angle:.0f}° (pulse {pulse_us} µs)")
        except Exception as e:
            self.get_logger().error(f"Erreur servo: {e}")

    # ══════════════════════════════════════════════════════════════════════
    # ODOMÉTRIE
    # ══════════════════════════════════════════════════════════════════════
    def update_odometry(self):
        left_curr  = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
        right_curr = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

        # Cohérence inversion/swap avec cmd_vel
        if self.swap_motors:
            left_curr, right_curr = right_curr, left_curr
        if self.invert_motors:
            left_curr  = -left_curr
            right_curr = -right_curr

        d_left  = (left_curr  - self.left_prev)  * self.meters_per_tick
        d_right = (right_curr - self.right_prev) * self.meters_per_tick
        self.left_prev  = left_curr
        self.right_prev = right_curr

        d = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x  = d / self.odom_dt
        odom.twist.twist.angular.z = d_theta / self.odom_dt
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # ══════════════════════════════════════════════════════════════════════
    # SHUTDOWN
    # ══════════════════════════════════════════════════════════════════════
    def destroy_node(self):
        try:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  0)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 0)
            self.gpg.reset_all()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoPiGoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
