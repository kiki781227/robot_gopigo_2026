import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_gopigo_msgs.msg import CubeDetect
import math 
import time

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # -------- PARAMETRES ---------
        self.declare_parameter('angle_rotation', 360.0)
        self.declare_parameter('vitesse_rotation', 1.0)
        self.declare_parameter('vitesse_approche', 0.15)
        self.declare_parameter('dist_aveugle', 0.25) 
        self.declare_parameter('gain_direction', 0.002) 

        # --------- VARIABLES D'ETAT --------
        self.phase = "DETECTION"
        self.cube_vu = False
        self.last_cube_msg = None
        self.pos_initiale_x = None
        self.x_entree_aveugle = 0.0
        self.debut_pause = None

        # --- COMMUNICATIONS ---
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10)

        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
            
        self.sub_cube = self.create_subscription(
            CubeDetect, 
            '/cube_detect', 
            self.cube_callback, 
            10)

        self.get_logger().info(f"Nœud prêt. En attente de détection...")

    def cube_callback(self, msg):
        self.last_cube_msg = msg
        self.cube_vu = msg.visible

    def odom_callback(self, msg):
        cmd = Twist()
        x_actuel = msg.pose.pose.position.x
        
        if self.pos_initiale_x is None:
            self.pos_initiale_x = x_actuel

        # --- MACHINE À ÉTATS ---

        # 1. Chercher le cube
        if self.phase == "DETECTION":
            if self.cube_vu:
                self.get_logger().info(f"Cube {self.last_cube_msg.color} repéré ! Alignement...")
                self.phase = "APPROCHE_ALIGNEE"
            else:
                cmd.angular.z = 0.3 

        # 2. Avancer (NEGATIF) vers le cube
        elif self.phase == "APPROCHE_ALIGNEE":
            if self.cube_vu:
                centre_img = self.last_cube_msg.img_width / 2
                erreur_x = self.last_cube_msg.cx - centre_img
                
                # Utilisation d'une valeur négative pour avancer
                cmd.linear.x = -abs(self.get_parameter('vitesse_approche').value)
                cmd.angular.z = - (self.get_parameter('gain_direction').value * erreur_x)
            else:
                self.get_logger().info("Cible proche. Passage en mode aveugle.")
                self.phase = "AVANCE_AVEUGLE"
                self.x_entree_aveugle = x_actuel

        # 3. Finir l'approche (NEGATIF) sans vision
        elif self.phase == "AVANCE_AVEUGLE":
            dist_parcourue = abs(x_actuel - self.x_entree_aveugle)
            if dist_parcourue < self.get_parameter('dist_aveugle').value:
                cmd.linear.x = -0.1 # On continue d'avancer en négatif
            else:
                self.get_logger().info("Arrivé au cube. Arrêt temporaire.")
                self.phase = "ATTENTE"
                self.debut_pause = time.time()

        # 4. S'arrêter (Simulation ramassage)
        elif self.phase == "ATTENTE":
            cmd.linear.x = 0.0
            if time.time() - self.debut_pause > 4.0:
                self.get_logger().info("Retour au point initial...")
                self.phase = "RETOUR"

        # 5. Retourner au départ (POSITIF)
        elif self.phase == "RETOUR":
            erreur_position = x_actuel - self.pos_initiale_x
            # On vérifie si on est encore loin de l'origine (seuil de 5cm)
            if abs(erreur_position) > 0.05: 
                cmd.linear.x = 0.2 # On utilise une valeur POSITIVE pour revenir
            else:
                self.get_logger().info("Point de départ atteint. Rotation finale.")
                self.phase = "ROTATION_FINALE"
                self.angle_parcouru = 0.0

        # 6. Rotation finale
        elif self.phase == "ROTATION_FINALE":
            # Ta logique de rotation ici
            cmd.angular.z = 1.0
            # ... condition de fin ...
            pass

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()