import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from gopigo3 import GoPiGo3

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Constantes de calibration
        # Les valeurs qui sont attribués aux variables TICKS_PAR_METRE et TICKS_PAR_TOUR indiquent combien de "ticks" (impulsions d'encodeur) sont générés 
        # par les moteurs pour parcourir un mètre ou effectuer une rotation complète. Ces nombres ne sont pas arbitraires, ils résultent de mesures expérimentales et des caractéristiques mécaniques du robot
        # Les encodeurs sont des capteurs intégrés aux moteurs qui comptent les rotations.
        # Chaque tick correspond à une fraction de tour de roue.
        TICKS_PAR_METRE = 1655
        TICKS_PAR_TOUR = 1130

        self.ENTRAXE = (2 * math.pi) / (TICKS_PAR_TOUR / TICKS_PAR_METRE)  # distance entre les roues en mètres
        
        # Conversion en mètre du nombre de ticks effectuer par le moteur lors de son déplacement.
        self.METRES_PAR_TICK = 1.0 / TICKS_PAR_METRE

        # GoPiGo3
        self.gpg = GoPiGo3()

        # Position initiale
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encodeurs précédents
        # self.left_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT) -> Cela va permettre de récupérer la valeur
        # spécifiant le nombre de ticks effectuer par le moteur gauche du robot
        self.left_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
        
        # self.right_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT) -> Cela va permettre de récupérer la valeur
        # spécifiant le nombre de ticks effectuer par le moteur droite du robot
        self.right_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

        # Publisher odométrie
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer 20 Hz
        self.timer = self.create_timer(0.05, self.update_odometry)
        self.get_logger().info('Nœud odométrie démarré !')

    def update_odometry(self):
        # Lire les encodeurs
        left_curr = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
        right_curr = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

        # Calculer les deltas
        d_left = (left_curr - self.left_prev) * self.METRES_PAR_TICK
        d_right = (right_curr - self.right_prev) * self.METRES_PAR_TICK

        self.left_prev = left_curr
        self.right_prev = right_curr

        # Calcul odométrie différentielle
        d = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.ENTRAXE

        self.theta += d_theta
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        # Publier odométrie
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        self.odom_pub.publish(odom)

        # Publier TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

	self.check_arena_bounds()
    def check_arena_bounds(self):
    	limit = 2.5 / 2.0  # 1.25 m

    	# Vérifie si le robot sort du carré
    	if abs(self.x) > limit or abs(self.y) > limit:
        	if not hasattr(self, "out_of_bounds") or not self.out_of_bounds:
           		self.get_logger().warn(f"Hors de l'arène ! x={self.x:.2f}, y={self.y:.2f}")
            		self.out_of_bounds = True
        	return True
    	else:
        	if hasattr(self, "out_of_bounds") and self.out_of_bounds:
            	self.get_logger().info("✅ Retour dans l'arène")
        	self.out_of_bounds = False
        	return False


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
