import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_gopigo_msgs.msg import CubeDetect, ArucoDetect
from sensor_msgs.msg import Range
import math
import time


# ─────────────────────────────────────────────
# Utilitaire : clamp
# ─────────────────────────────────────────────
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


class MissionNode(Node):

    # ──────────────────────────────────────────
    # INITIALISATION
    # ──────────────────────────────────────────
    def __init__(self):
        super().__init__('mission_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('vitesse_rotation',  0.25)   # rad/s  (était 0.1, trop lent/rapide selon robot)
        self.declare_parameter('vitesse_approche',  0.10)   # m/s    (réduit pour mieux centrer)
        self.declare_parameter('vitesse_retour',    0.15)   # m/s
        self.declare_parameter('dist_aveugle',      0.25)   # m      (distance finale sans vision)
        self.declare_parameter('gain_direction',    1.5e-3) # rad/px (légèrement réduit)
        self.declare_parameter('gain_direction_max',0.30)   # rad/s  clamp correction angulaire
        self.declare_parameter('seuil_centrage',    25.0)   # px     (plus strict qu'avant)
        self.declare_parameter('dist_arret_mur',    0.18)   # m

        # Mapping couleur → ID ArUco dépôt
        self.declare_parameter('couleur_vers_aruco.rouge',  0)
        self.declare_parameter('couleur_vers_aruco.bleu',   1)
        self.declare_parameter('couleur_vers_aruco.vert',   1)
        self.declare_parameter('couleur_vers_aruco.jaune',  0)

        self.declare_parameter('max_cycles', 3)

        # ── Variables d'état ────────────────────────────────────────────────
        self.phase = "DETECTION"
        self.cycles_effectues = 0
        self._phase_precedente = ""   # pour n'afficher les logs qu'au changement de phase

        # Capteurs
        self.cube_vu        = False
        self.last_cube_msg  = None
        self.aruco_vu       = False
        self.last_aruco_msg = None
        self.distance_mur   = 4.0   # Valeur initiale haute = "pas d'obstacle"

        # Couleur du cube en cours de transport
        self.couleur_cube_ramasse = None

        # Odométrie
        self.pos_initiale_x  = None
        self.pos_initiale_y  = None
        self.x_entree_aveugle = 0.0
        self.y_entree_aveugle = 0.0
        self.debut_pause      = None

        # Historique de trajectoire pour l'extraction du dépôt
        # On stocke des tuples (x, y) à chaque tick d'approche
        self.historique_pos_depot = []
        self._derniere_pos_enregistree = None
        self._dist_min_enregistrement = 0.02  # Enregistre 1 point tous les ~2 cm

        # Compteur pour limiter la fréquence des logs répétitifs
        self._tick_count = 0
        self._log_interval = 20  # log tous les 20 ticks (~1s à 20 Hz)

        # ── Communications ──────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_cube = self.create_subscription(
            CubeDetect, '/cube_detect', self.cube_callback, 10)

        self.sub_aruco = self.create_subscription(
            ArucoDetect, '/aruco_detect', self.aruco_callback, 10)

        self.sub_ultrasonic = self.create_subscription(
            Range, '/ultrasonic/front', self.ultrasonic_callback, 10)

        # ── Timer de mission (découplé de l'odométrie) ──────────────────────
        # La logique tourne à 10 Hz, indépendamment du topic /odom.
        # L'odométrie ne fait que mettre à jour self.x_actuel / self.y_actuel.
        self.x_actuel = 0.0
        self.y_actuel = 0.0
        self.mission_timer = self.create_timer(0.1, self.mission_step)

        self.get_logger().info("Nœud mission prêt — attente de détection.")

    # ──────────────────────────────────────────
    # CALLBACKS CAPTEURS
    # ──────────────────────────────────────────
    def odom_callback(self, msg):
        """Met uniquement à jour la position courante."""
        self.x_actuel = msg.pose.pose.position.x
        self.y_actuel = msg.pose.pose.position.y
        # Mémorise la position de départ au premier message
        if self.pos_initiale_x is None:
            self.pos_initiale_x = self.x_actuel
            self.pos_initiale_y = self.y_actuel
            self.get_logger().info(
                f"Position initiale mémorisée : x={self.pos_initiale_x:.3f}, y={self.pos_initiale_y:.3f}")

    def cube_callback(self, msg):
        self.last_cube_msg = msg
        self.cube_vu = msg.visible

    def aruco_callback(self, msg):
        self.last_aruco_msg = msg
        self.aruco_vu = msg.visible

    def ultrasonic_callback(self, msg):
        self.distance_mur = msg.range

    # ──────────────────────────────────────────
    # UTILITAIRES
    # ──────────────────────────────────────────
    def _dist_au_depart(self) -> float:
        """Distance euclidienne entre la position actuelle et le point de départ."""
        if self.pos_initiale_x is None:
            return 0.0
        dx = self.x_actuel - self.pos_initiale_x
        dy = self.y_actuel - self.pos_initiale_y
        return math.sqrt(dx * dx + dy * dy)

    def _changer_phase(self, nouvelle_phase: str):
        """Change de phase et log le changement."""
        self.get_logger().info(
            f"Phase : {self.phase}  →  {nouvelle_phase}")
        self.phase = nouvelle_phase

    def _log_phase(self, msg: str = ""):
        """Log dans la phase courante, uniquement tous les N ticks."""
        self._tick_count += 1
        if self._tick_count % self._log_interval == 0:
            if msg:
                self.get_logger().info(f"[{self.phase}] {msg}")
            else:
                self.get_logger().info(f"[{self.phase}]")

    def _correction_angulaire(self, erreur_px: float) -> float:
        """Calcule et clampe la correction angulaire en rad/s."""
        gain = self.get_parameter('gain_direction').value
        max_w = self.get_parameter('gain_direction_max').value
        return clamp(-gain * erreur_px, -max_w, max_w)

    def _erreur_centre(self, msg) -> float:
        """Erreur en px entre le centre de l'objet détecté et le centre image."""
        return msg.cx - (msg.img_width / 2.0)

    def _id_depot_pour_couleur(self) -> int:
        """Retourne l'ID ArUco correspondant à la couleur du cube ramassé."""
        nom_param = f'couleur_vers_aruco.{self.couleur_cube_ramasse}'
        try:
            return self.get_parameter(nom_param).value
        except Exception:
            self.get_logger().warn(
                f"Couleur inconnue : '{self.couleur_cube_ramasse}'. Utilisation ID 0 par défaut.")
            return 0

    # ──────────────────────────────────────────
    # BOUCLE PRINCIPALE DE MISSION
    # ──────────────────────────────────────────
    def mission_step(self):
        """
        Appelée par le timer à 10 Hz.
        Contient la machine à états complète.
        """
        if self.pos_initiale_x is None:
            return   # Attente du premier message odométrie

        cmd = Twist()  # Par défaut : tout à zéro

        # ── 1. DETECTION ──────────────────────────────────────────────────
        if self.phase == "DETECTION":
            if self.cube_vu:
                self.couleur_cube_ramasse = str(self.last_cube_msg.color).lower()
                self.get_logger().info(
                    f"Cube '{self.last_cube_msg.color}' repéré ! Début du centrage.")
                self._changer_phase("CENTRAGE")
            else:
                # Rotation lente
                cmd.angular.z = self.get_parameter('vitesse_rotation').value
                self._log_phase("rotation en cours...")

        # ── 2. CENTRAGE ───────────────────────────────────────────────────
        elif self.phase == "CENTRAGE":
            if self.cube_vu:
                erreur_x = self._erreur_centre(self.last_cube_msg)
                seuil    = self.get_parameter('seuil_centrage').value

                if abs(erreur_x) > seuil:
                    # Rotation seule — aucune translation
                    cmd.angular.z = self._correction_angulaire(erreur_x)
                    cmd.linear.x  = 0.0
                    self._log_phase(f"erreur={erreur_x:.0f}px, ω={cmd.angular.z:.3f}")
                else:
                    self.get_logger().info(
                        f"Cube centré (erreur={erreur_x:.1f}px). Départ avance.")
                    self._changer_phase("APPROCHE_ALIGNEE")
            else:
                self.get_logger().info("Cube perdu pendant centrage → retour détection.")
                self._changer_phase("DETECTION")

        # ── 3. APPROCHE ALIGNÉE ───────────────────────────────────────────
        elif self.phase == "APPROCHE_ALIGNEE":
            if self.cube_vu:
                erreur_x = self._erreur_centre(self.last_cube_msg)
                seuil    = self.get_parameter('seuil_centrage').value

                # Avance lente
                cmd.linear.x = -abs(self.get_parameter('vitesse_approche').value)

                # Correction angulaire réduite (on avance, ne pas surcompenser)
                if abs(erreur_x) > seuil:
                    cmd.angular.z = self._correction_angulaire(erreur_x) * 0.5
                else:
                    cmd.angular.z = 0.0

                self._log_phase(f"erreur={erreur_x:.0f}px, vx={cmd.linear.x:.2f}")
            else:
                self.get_logger().info("Cube sorti du champ → avance aveugle.")
                self.x_entree_aveugle = self.x_actuel
                self.y_entree_aveugle = self.y_actuel
                self._changer_phase("AVANCE_AVEUGLE")

        # ── 4. AVANCE AVEUGLE ─────────────────────────────────────────────
        elif self.phase == "AVANCE_AVEUGLE":
            dx = self.x_actuel - self.x_entree_aveugle
            dy = self.y_actuel - self.y_entree_aveugle
            dist_parcourue = math.sqrt(dx * dx + dy * dy)

            if dist_parcourue < self.get_parameter('dist_aveugle').value:
                cmd.linear.x = -0.08   # Avance encore plus lentement
                self._log_phase(f"dist={dist_parcourue:.3f}m")
            else:
                self._changer_phase("ATTENTE_RAMASSAGE")
                self.debut_pause = time.time()

        # ── 5. ATTENTE RAMASSAGE ──────────────────────────────────────────
        elif self.phase == "ATTENTE_RAMASSAGE":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            if time.time() - self.debut_pause > 4.0:
                self.get_logger().info("Cube ramassé (simulé). Retour au départ.")
                self._changer_phase("RETOUR_POST_RAMASSAGE")

        # ── 6. RETOUR POST RAMASSAGE ──────────────────────────────────────
        elif self.phase == "RETOUR_POST_RAMASSAGE":
            dist = self._dist_au_depart()
            if dist > 0.06:
                # Direction vers le point de départ
                dx = self.pos_initiale_x - self.x_actuel
                dy = self.pos_initiale_y - self.y_actuel
                angle_cible = math.atan2(dy, dx)
                cmd.linear.x  = abs(self.get_parameter('vitesse_retour').value)
                # Petite correction cap (simplifié — pour un retour en ligne droite)
                cmd.angular.z = clamp(angle_cible * 0.5, -0.3, 0.3)
                self._log_phase(f"dist_départ={dist:.3f}m")
            else:
                self.get_logger().info("Retour départ OK. Recherche du dépôt ArUco.")
                self._changer_phase("RECHERCHE_DEPOT")

        # ── 7. RECHERCHE DÉPÔT ───────────────────────────────────────────
        elif self.phase == "RECHERCHE_DEPOT":
            id_cible = self._id_depot_pour_couleur()

            if self.aruco_vu and self.last_aruco_msg.marker_id == id_cible:
                self.get_logger().info(
                    f"Dépôt ID {id_cible} trouvé ({self.couleur_cube_ramasse}). Pause 1.5s.")
                self.historique_pos_depot = []
                self._derniere_pos_enregistree = None
                self._changer_phase("PAUSE_AVANT_APPROCHE")
                self.debut_pause = time.time()
            else:
                cmd.angular.z = self.get_parameter('vitesse_rotation').value
                self._log_phase(f"rotation, cherche ArUco {id_cible}...")

        # ── 8. PAUSE AVANT APPROCHE ───────────────────────────────────────
        elif self.phase == "PAUSE_AVANT_APPROCHE":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            if time.time() - self.debut_pause > 1.5:
                self.get_logger().info("Début approche dépôt.")
                self._changer_phase("APPROCHE_DEPOT")

        # ── 9. APPROCHE DÉPÔT ─────────────────────────────────────────────
        elif self.phase == "APPROCHE_DEPOT":
            dist_mur = self.distance_mur

            # ── Sécurité : mur trop proche ────────────────────────────────
            if dist_mur <= self.get_parameter('dist_arret_mur').value:
                self.get_logger().info(
                    f"Mur détecté à {dist_mur:.2f}m → arrêt et dépôt.")
                self._changer_phase("DEPOT_CUBE")
                self.debut_pause = time.time()
            else:
                # Enregistrement de la position tous les ~2 cm pour extraction
                pos_courante = (self.x_actuel, self.y_actuel)
                if self._derniere_pos_enregistree is None:
                    self.historique_pos_depot.append(pos_courante)
                    self._derniere_pos_enregistree = pos_courante
                else:
                    dx = self.x_actuel - self._derniere_pos_enregistree[0]
                    dy = self.y_actuel - self._derniere_pos_enregistree[1]
                    if math.sqrt(dx*dx + dy*dy) >= self._dist_min_enregistrement:
                        self.historique_pos_depot.append(pos_courante)
                        self._derniere_pos_enregistree = pos_courante

                cmd.linear.x = -abs(self.get_parameter('vitesse_approche').value)

                if self.aruco_vu:
                    erreur_x_aruco = self._erreur_centre(self.last_aruco_msg)
                    cmd.angular.z  = self._correction_angulaire(erreur_x_aruco) * 0.5
                else:
                    cmd.angular.z = 0.0
                    self._log_phase("ArUco non visible, avance sans correction.")

        # ── 10. DÉPÔT CUBE ─────────────────────────────────────────────────
        elif self.phase == "DEPOT_CUBE":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            if time.time() - self.debut_pause > 2.0:
                self.get_logger().info("Cube déposé. Extraction du dépôt.")
                self._changer_phase("EXTRACTION_DEPOT")

        # ── 11. EXTRACTION DÉPÔT ──────────────────────────────────────────
        # On rejoue le chemin à rebours (waypoint par waypoint)
        elif self.phase == "EXTRACTION_DEPOT":
            if self.historique_pos_depot:
                cible_x, cible_y = self.historique_pos_depot[-1]
                dx = cible_x - self.x_actuel
                dy = cible_y - self.y_actuel
                dist_cible = math.sqrt(dx*dx + dy*dy)

                if dist_cible > 0.04:
                    # Se dirige vers le waypoint
                    angle_vers_cible = math.atan2(dy, dx)
                    cmd.linear.x  = abs(self.get_parameter('vitesse_approche').value)
                    cmd.angular.z = clamp(angle_vers_cible * 0.5, -0.3, 0.3)
                    self._log_phase(f"extraction vers waypoint, dist={dist_cible:.3f}m")
                else:
                    # Waypoint atteint, on passe au suivant
                    self.historique_pos_depot.pop()
            else:
                self.get_logger().info("Extraction terminée. Retour base final.")
                self._changer_phase("RETOUR_BASE_FINAL")

        # ── 12. RETOUR BASE FINAL ─────────────────────────────────────────
        elif self.phase == "RETOUR_BASE_FINAL":
            dist = self._dist_au_depart()
            if dist > 0.06:
                dx = self.pos_initiale_x - self.x_actuel
                dy = self.pos_initiale_y - self.y_actuel
                angle_cible = math.atan2(dy, dx)
                cmd.linear.x  = abs(self.get_parameter('vitesse_retour').value)
                cmd.angular.z = clamp(angle_cible * 0.5, -0.3, 0.3)
                self._log_phase(f"dist_départ={dist:.3f}m")
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.cycles_effectues += 1
                self.get_logger().info(f"Cycle {self.cycles_effectues} terminé.")

                if self.cycles_effectues < self.get_parameter('max_cycles').value:
                    self.get_logger().info("Nouveau cycle en cours...")
                    self.cube_vu  = False
                    self.aruco_vu = False
                    self._changer_phase("DETECTION")
                else:
                    self.get_logger().info("Nombre max de cycles atteint. Mission terminée.")
                    self._changer_phase("FINAL_PROCESS")

        # ── 13. FINAL PROCESS ─────────────────────────────────────────────
        elif self.phase == "FINAL_PROCESS":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        # ── Publication ───────────────────────────────────────────────────
        self.publisher.publish(cmd)


# ──────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────
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
