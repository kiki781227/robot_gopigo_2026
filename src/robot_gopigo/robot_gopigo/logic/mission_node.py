import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_gopigo_msgs.msg import CubeDetect, ArucoDetect
import math 
import time

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # -------- PARAMETRES ---------
        self.declare_parameter('vitesse_rotation', 1.0)
        self.declare_parameter('vitesse_approche', 0.15)
        self.declare_parameter('dist_aveugle', 0.25) 
        self.declare_parameter('gain_direction', 0.002) 
        # self.declare_parameter('id_depot', 1)
        self.declare_parameter('max_cycles', 3) # Nombre de répétitions max 
        self.declare_parameter('seuil_distance_limite', 120.0) # Seuil de taille pour s'arrêter près du mur

        # --------- VARIABLES D'ETAT --------
        self.phase = "DETECTION"
        self.cycle_effectues = 0

        # Capteurs
        self.cube_vu = False    
        self.last_cube_msg = None
        self.aruco_vu = False
        self.last_aruco_msg = None

        # Mémoire de mouvement
        self.historique_x_aruco = []
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

        self.sub_aruco = self.create_subscription(
            ArucoDetect,
            '/aruco_detect',
            self.aruco_callback,
            10)

        self.get_logger().info(f"Nœud prêt. En attente de détection...")

    def cube_callback(self, msg):
        self.last_cube_msg = msg
        self.cube_vu = msg.visible

    def aruco_callback(self, msg):
        self.last_aruco_msg = msg # Pour 
        self.aruco_vu = msg.visible

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
                self.phase = "ATTENTE_RAMASSAGE"
                self.debut_pause = time.time()

        # 4. S'arrêter (Simulation ramassage)
        elif self.phase == "ATTENTE_RAMASSAGE":
            cmd.linear.x = 0.0
            if time.time() - self.debut_pause > 4.0:
                self.get_logger().info("Cube ramassé. Retour au point initial...")
                self.phase = "RETOUR_POST_RAMASSAGE"

        # 5. Retourner au départ (POSITIF)
        elif self.phase == "RETOUR_POST_RAMASSAGE":
            erreur_position = x_actuel - self.pos_initiale_x
            # On vérifie si on est encore loin de l'origine (seuil de 5cm)
            if abs(erreur_position) > 0.05: 
                cmd.linear.x = 0.2 # On utilise une valeur POSITIVE pour revenir
            else:
                self.get_logger().info("Point de départ atteint. Recherche du dépôt ArUco...")
                self.phase = "RECHERCHE_DEPOT"

        # 6. Rotation finale
        elif self.phase == "RECHERCHE_DEPOT":
            #id_cible = self.get_parameter('id_depot').value
            id_cible = 1

            if self.aruco_vu and self.last_aruco_msg.marker_id == id_cible:
                self.get_logger().info(f"Dépôt ID {id_cible} trouvé ! Pause de 1.5s.")
                self.phase = "PAUSE_AVANT_APPROCHE"
                self.debut_pause = time.time()
                self.historique_x_aruco = []
            else:
                cmd.angular.z = 0.3

        # 7. Pause 1.5s demandée
        elif self.phase == "PAUSE_AVANT_APPROCHE":
            cmd.linear.x = 0.0
            if time.time() - self.debut_pause > 1.5:
                self.phase == "APPROCHE_DEPOT"

        # 8. Approche dépôt + Stockage tableau
        elif self.phase == "APPROCHE_DEPOT":
            self.historique_x_aruco.append(x_actuel)
            if self.aruco_vu:
                if self.last_aruco_msg.size >= self.get_parameter('seuil_distance_limite').value:
                    self.phase = "DEPOT_CUBE"
                    self.debut_pause = time.time()
                else:
                    # Cela va nous permettre de corriger la rotation pour permettre au robot 
                    # de se placer bien en face du dépot
                    centre_img_aruco = self.last_aruco_msg.img_width / 2
                    erreur_x_aruco = self.last_aruco_msg.cx - centre_img_aruco
                    cmd.linear.x = -abs(self.get_parameter('vitesse_approche').value)
                    cmd.angular.z = - (self.get_parameter('gain_direction').value * erreur_x_aruco)
                
            else:
                cmd.linear.x = 0.0
                self.get_logger().info("Dépôt non visible. Arrêt.")

        # 9. Pause dépôt
        elif self.phase == "DEPOT_CUBE":
            cmd.linear.x = 0.0
            if time.time() - self.debut_pause > 2.0:
                self.phase = "EXTRACTION_DEPOT"

        # 10.Extraction via tableau (pour s'éloigner du dépot)
        elif self.phase == "EXTRACTION_DEPOT":
            if len(self.historique_x_aruco) > 0:
                derniere_valeur = self.historique_x_aruco.pop()
                if x_actuel < derniere_valeur:
                    cmd.linear.x = abs(self.get_parameter('vitesse_approche').value)

                else:
                    cmd.linear.x = 0.0
            
            else:
                # Une fois sorti de la zone ArUco, on utilise l'Odométrie pour le retour final
                self.phase = "RETOUR_BASE_FINAL"

        # 11. Retour précis au point de départ (Logique Erreur Odométrie)
        elif self.phase == "RETOUR_BASE_FINAL":
            erreur_position = x_actuel - self.pos_initiale_x
            if abs(erreur_position) > 0.05:
                cmd.linear_x = 0.2 # Recul vers l'origine

            else:
                cmd.linear.x = 0.0
                self.cycles_effectues += 1
                self.get_logger().info(f"Cycle {self.cycles_effectues} terminé.")

                # Vérification du nombre de cycles
                if self.cycles_effectues < self.get_parameter('max_cycles').value:
                    self.get_logger().info("Redémarrage d'un nouveau cycle...")
                    self.phase = "DETECTION" # On recommence

                    # Reset des variables de détection pour le nouveau tour
                    self.cube_vu = False
                    self.aruco_vu = False

                else:
                    self.get_logger().info("Nombre maximum de cycles atteint. Mission terminée.")
                    self.phase = "FINAL_PROCESS"

        # 12. Fin de mission
        elif self.phase == "FINAL_PROCESS":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

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