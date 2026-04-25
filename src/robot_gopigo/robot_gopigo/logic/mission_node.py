"""
mission_node.py
───────────────
Stratégie de mission pour le robot GoPiGo3 trieur de cubes.

VERSION MODIFIÉE :
  - APPROCHER_CUBE : avance À L'AVEUGLE pendant N secondes (pas de centrage continu)
  - SAISIR : ouvre servo (cube tombe dans U) → attend → ferme servo → recule

Machine à états :
  INIT
    ↓
  CHERCHER_CUBE → CENTRAGE_CUBE → APPROCHER_CUBE (aveugle) → SAISIR
    ↓
  CHERCHER_DEPOT → CENTRAGE_DEPOT → APPROCHER_DEPOT → DEPOSER → RECULER → CHERCHER_CUBE
"""

import math
import time
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from robot_gopigo_msgs.msg import CubeDetect, ArucoDetect


# ─── Enum des états ────────────────────────────────────────────────────────
class State(Enum):
    INIT             = "INIT"
    CHERCHER_CUBE      = "CHERCHER_CUBE"
    CENTRAGE_CUBE      = "CENTRAGE_CUBE"
    APPROCHER_CUBE     = "APPROCHER_CUBE"
    APPROCHE_FINALE    = "APPROCHE_FINALE"
    SAISIR             = "SAISIR"
    RETOUR_CENTRE      = "RETOUR_CENTRE"
    CHERCHER_DEPOT     = "CHERCHER_DEPOT"
    CENTRAGE_DEPOT     = "CENTRAGE_DEPOT"
    APPROCHER_DEPOT    = "APPROCHER_DEPOT"
    DEPOSER            = "DEPOSER"
    RECULER            = "RECULER"
    FIN                = "FIN"
    FIN_ERREUR         = "FIN_ERREUR"


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ── Paramètres ROS ──────────────────────────────────────────────────
        self.declare_parameter('search_rotation_speed',  0.3)
        self.declare_parameter('approach_linear_speed',  0.15)
        self.declare_parameter('approach_deposit_speed', 0.12)
        self.declare_parameter('backup_speed',           0.15)

        self.declare_parameter('cube_centering_gain',    0.003)
        self.declare_parameter('aruco_centering_gain',   0.003)
        self.declare_parameter('centering_threshold',    25)

        self.declare_parameter('detection_stable_frames', 3)
        self.declare_parameter('detection_lost_frames',   10)

        self.declare_parameter('rotation_mode',           'continuous')
        self.declare_parameter('pulse_rotation_duration', 0.4)
        self.declare_parameter('pulse_pause_duration',    0.6)

        self.declare_parameter('deposit_stop_distance',  0.22)
        self.declare_parameter('cube_approach_min_area', 5000)
        self.declare_parameter('backup_distance',        0.25)

        self.declare_parameter('timeout_chercher_cube',  45.0)
        self.declare_parameter('timeout_chercher_depot', 30.0)
        self.declare_parameter('timeout_approcher',      15.0)

        self.declare_parameter('grab_duration',          1.5)
        self.declare_parameter('release_duration',       1.5)

        # ── NOUVEAUX PARAMÈTRES ─────────────────────────────────────────────
        # Durée de l'avance aveugle (APPROCHER_CUBE)
        self.declare_parameter('approach_blind_duration', 2.5)
        # Pause entre ouverture du servo et fermeture (laisse le cube tomber dans U)
        self.declare_parameter('grab_settle_duration',    1.5)
        # Durée de la marche arrière après SAISIR pour revenir vers le centre
        self.declare_parameter('retour_centre_duration',  3.0)

        self.declare_parameter('servo_angle_open',       180)
        self.declare_parameter('servo_angle_close',      0)

        self.declare_parameter('color_to_aruco_id.rouge', 0)
        self.declare_parameter('color_to_aruco_id.jaune', 0)
        self.declare_parameter('color_to_aruco_id.bleu',  2)
        self.declare_parameter('color_to_aruco_id.vert',  2)

        self.declare_parameter('ignored_colors', [''])
        self.declare_parameter('max_cycles',             10)
        self.declare_parameter('state_machine_rate',     20.0)

        # ── Lecture des paramètres ──────────────────────────────────────────
        self.search_rot_speed     = self.get_parameter('search_rotation_speed').value
        self.approach_speed       = self.get_parameter('approach_linear_speed').value
        self.approach_dep_speed   = self.get_parameter('approach_deposit_speed').value
        self.backup_speed         = self.get_parameter('backup_speed').value

        self.cube_gain            = self.get_parameter('cube_centering_gain').value
        self.aruco_gain           = self.get_parameter('aruco_centering_gain').value
        self.center_threshold     = self.get_parameter('centering_threshold').value

        self.stable_frames        = self.get_parameter('detection_stable_frames').value
        self.lost_frames          = self.get_parameter('detection_lost_frames').value

        self.rotation_mode        = self.get_parameter('rotation_mode').value
        self.pulse_rot_dur        = self.get_parameter('pulse_rotation_duration').value
        self.pulse_pause_dur      = self.get_parameter('pulse_pause_duration').value

        self.deposit_stop_dist    = self.get_parameter('deposit_stop_distance').value
        self.cube_min_area        = self.get_parameter('cube_approach_min_area').value
        self.backup_dist          = self.get_parameter('backup_distance').value

        self.to_chercher_cube     = self.get_parameter('timeout_chercher_cube').value
        self.to_chercher_depot    = self.get_parameter('timeout_chercher_depot').value
        self.to_approcher         = self.get_parameter('timeout_approcher').value

        self.grab_duration        = self.get_parameter('grab_duration').value
        self.release_duration     = self.get_parameter('release_duration').value

        # NOUVEAUX
        self.approach_blind_dur   = self.get_parameter('approach_blind_duration').value
        self.grab_settle_dur      = self.get_parameter('grab_settle_duration').value
        self.retour_centre_dur    = self.get_parameter('retour_centre_duration').value

        self.servo_open           = self.get_parameter('servo_angle_open').value
        self.servo_close          = self.get_parameter('servo_angle_close').value

        self.color_map = {
            'rouge': self.get_parameter('color_to_aruco_id.rouge').value,
            'jaune': self.get_parameter('color_to_aruco_id.jaune').value,
            'bleu':  self.get_parameter('color_to_aruco_id.bleu').value,
            'vert':  self.get_parameter('color_to_aruco_id.vert').value,
        }
        self.ignored_colors = [c for c in self.get_parameter('ignored_colors').value if c]

        self.max_cycles           = self.get_parameter('max_cycles').value
        self.sm_rate              = self.get_parameter('state_machine_rate').value

        # ── État de la machine ──────────────────────────────────────────────
        self.state       = State.INIT
        self.state_start = time.time()
        self.cycles_done = 0

        self.last_cube      = None
        self.last_aruco     = None
        self.last_odom      = None
        self.cube_stable    = 0
        self.cube_lost      = 0
        self.aruco_stable   = 0
        self.aruco_lost     = 0

        smoothing_window = 5
        self.cube_cx_buffer  = deque(maxlen=smoothing_window)
        self.aruco_cx_buffer = deque(maxlen=smoothing_window)

        self.locked_color   = None
        self.target_aruco   = -1

        self.backup_start_x = 0.0
        self.backup_start_y = 0.0
        self.action_start_time = 0.0

        # NOUVEAU : sous-état pour SAISIR (séquence ouvert→pause→fermé→recule)
        self.saisir_phase = 'OUVRIR'  # 'OUVRIR' → 'ATTENDRE_OUVERT' → 'FERMER' → 'ATTENDRE_FERME'

        # ── ROS interfaces ──────────────────────────────────────────────────
        self.pub_cmd   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_servo = self.create_publisher(Float32, '/servo_cmd', 10)
        self.pub_state = self.create_publisher(String, '/mission_state', 10)

        self.create_subscription(Odometry,   '/odom',         self._odom_cb,  10)
        self.create_subscription(CubeDetect, '/cube_detect',  self._cube_cb,  10)
        self.create_subscription(ArucoDetect,'/aruco_detect', self._aruco_cb, 10)

        self.sm_timer = self.create_timer(1.0 / self.sm_rate, self.step)
        self.state_pub_timer = self.create_timer(0.5, self._publish_state)

        # Servo FERMÉ au démarrage (s'ouvrira dès qu'un cube est détecté)
        self._send_servo(self.servo_close)

        self.get_logger().info("Mission node prêt - démarrage dans 2s...")
        self.start_time = time.time()
        self.started = False

    # ══════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════
    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def _cube_cb(self, msg: CubeDetect):
        self.last_cube = msg
        if msg.visible and self._is_cube_valid(msg):
            self.cube_stable += 1
            self.cube_lost = 0
            self.cube_cx_buffer.append(msg.cx)
        else:
            self.cube_stable = 0
            self.cube_lost += 1
            if self.cube_lost > 3:
                self.cube_cx_buffer.clear()

    def _aruco_cb(self, msg: ArucoDetect):
        self.last_aruco = msg
        if msg.visible:
            self.aruco_stable += 1
            self.aruco_lost = 0
            self.aruco_cx_buffer.append(msg.cx)
        else:
            self.aruco_stable = 0
            self.aruco_lost += 1
            if self.aruco_lost > 3:
                self.aruco_cx_buffer.clear()

    # ══════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════
    def _is_cube_valid(self, cube: CubeDetect) -> bool:
        if not cube.visible:
            return False
        if cube.color not in self.color_map:
            return False
        if cube.color in self.ignored_colors:
            return False
        if self.locked_color is not None and cube.color != self.locked_color:
            return False
        return True

    def _is_aruco_target(self, aruco: ArucoDetect) -> bool:
        if not aruco.visible:
            return False
        return aruco.marker_id == self.target_aruco

    def _change_state(self, new_state: State):
        self.get_logger().info(f"STATE: {self.state.value} → {new_state.value}")
        self.state = new_state
        self.state_start = time.time()
        self.cube_stable = 0
        self.cube_lost = 0
        self.aruco_stable = 0
        self.aruco_lost = 0
        # Reset sous-état SAISIR à chaque changement
        self.saisir_phase = 'OUVRIR'

    def _time_in_state(self):
        return time.time() - self.state_start

    def _send_cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def _send_servo(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.pub_servo.publish(msg)

    def _publish_state(self):
        color = self.locked_color if self.locked_color else '-'
        aruco = self.target_aruco if self.target_aruco != -1 else '-'
        text = (
            f"{self.state.value}"
            f"|cycle={self.cycles_done}/{self.max_cycles}"
            f"|color={color}"
            f"|aruco={aruco}"
        )
        msg = String()
        msg.data = text
        self.pub_state.publish(msg)

    def _stop(self):
        self._send_cmd(0.0, 0.0)

    def _get_pose(self):
        if self.last_odom is None:
            return None, None, None
        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        return p.x, p.y, theta

    def _distance_from(self, x0, y0):
        x, y, _ = self._get_pose()
        if x is None:
            return 0.0
        return math.hypot(x - x0, y - y0)

    def _search_rotation_cmd(self, base_speed):
        if self.rotation_mode != 'pulsed':
            return base_speed

        cycle_dur = self.pulse_rot_dur + self.pulse_pause_dur
        t_in_cycle = self._time_in_state() % cycle_dur

        if t_in_cycle < self.pulse_rot_dur:
            return base_speed
        else:
            return 0.0

    # ══════════════════════════════════════════════════════════════════════
    # STEP : boucle principale
    # ══════════════════════════════════════════════════════════════════════
    def step(self):
        if not self.started:
            if time.time() - self.start_time < 2.0:
                return
            self.started = True
            self._change_state(State.CHERCHER_CUBE)
            return

        handlers = {
            State.CHERCHER_CUBE:    self._s_chercher_cube,
            State.CENTRAGE_CUBE:    self._s_centrage_cube,
            State.APPROCHER_CUBE:   self._s_approcher_cube,
            State.APPROCHE_FINALE:  self._s_approche_finale,
            State.SAISIR:           self._s_saisir,
            State.RETOUR_CENTRE:    self._s_retour_centre,
            State.CHERCHER_DEPOT:   self._s_chercher_depot,
            State.CENTRAGE_DEPOT:   self._s_centrage_depot,
            State.APPROCHER_DEPOT:  self._s_approcher_depot,
            State.DEPOSER:          self._s_deposer,
            State.RECULER:          self._s_reculer,
            State.FIN:              self._s_fin,
            State.FIN_ERREUR:       self._s_fin_erreur,
        }
        handler = handlers.get(self.state)
        if handler:
            handler()

    # ══════════════════════════════════════════════════════════════════════
    # ÉTATS
    # ══════════════════════════════════════════════════════════════════════
    def _s_chercher_cube(self):
        if self._time_in_state() > self.to_chercher_cube:
            self.get_logger().warn("Timeout CHERCHER_CUBE - arrêt")
            self._change_state(State.FIN_ERREUR)
            return

        if self.cube_stable >= self.stable_frames and self._is_cube_valid(self.last_cube):
            self.locked_color = self.last_cube.color
            self.target_aruco = self.color_map[self.locked_color]
            # OUVRIR LE SERVO IMMÉDIATEMENT
            self._send_servo(self.servo_open)
            self.get_logger().info(
                f"Cube '{self.locked_color}' détecté → SERVO OUVERT → CENTRAGE_CUBE "
                f"(cible ArUco={self.target_aruco})"
            )
            self._change_state(State.CENTRAGE_CUBE)
            return

        rot_speed = self._search_rotation_cmd(self.search_rot_speed)
        self._send_cmd(0.0, rot_speed)

    def _s_centrage_cube(self):
        """Centrage CONTINU sur le cube avec rotation proportionnelle.
        
        Phases :
          1. PAUSE INITIALE (1s) : robot immobile, on observe le cube
                                    sans bouger pour bien stabiliser la détection
          2. ROTATION PROPORTIONNELLE : tourne lentement vers le cube
                                         vitesse proportionnelle à l'erreur
          3. CONFIRMATION : 5 frames stables centré → APPROCHER
        """
        # Timeout
        if self._time_in_state() > self.to_approcher * 2:
            self.get_logger().warn("Timeout CENTRAGE_CUBE - retour CHERCHER")
            self._reset_cycle()
            self._change_state(State.CHERCHER_CUBE)
            return

        # Perte prolongée → retour à la recherche
        if self.cube_lost >= self.lost_frames:
            self.get_logger().warn("Cube perdu pendant centrage - retour CHERCHER")
            self._reset_cycle()
            self._change_state(State.CHERCHER_CUBE)
            return

        # ─── PHASE 1 : PAUSE INITIALE (1s) ─────────────────────────────────
        # Robot immobile pour bien stabiliser l'image et la détection
        pause_initial = 1.0
        if self._time_in_state() < pause_initial:
            self._stop()
            return

        # Cube non visible cette frame : on attend, robot immobile
        if not (self.last_cube and self.last_cube.visible
                and self._is_cube_valid(self.last_cube)):
            self._stop()
            return

        # ─── PHASE 2 : Calcul de l'erreur (cx moyenné) ─────────────────────
        img_center = self.last_cube.img_width / 2.0
        if len(self.cube_cx_buffer) >= 2:
            avg_cx = sum(self.cube_cx_buffer) / len(self.cube_cx_buffer)
        else:
            avg_cx = self.last_cube.cx
        error_x = avg_cx - img_center

        # ─── PHASE 3 : Centré stable ? ─────────────────────────────────────
        if abs(error_x) < self.center_threshold:
            self._centrage_stable_count = getattr(self, '_centrage_stable_count', 0) + 1
            self._stop()
            # Stable pendant 5 frames → vraiment centré
            if self._centrage_stable_count >= 5:
                self.get_logger().info(
                    f"Cube centré stable (err={error_x:.0f}px) → APPROCHER_CUBE"
                )
                self._centrage_stable_count = 0
                self._change_state(State.APPROCHER_CUBE)
            return

        # Pas centré → reset compteur stable
        self._centrage_stable_count = 0

        # ─── ROTATION SIMPLE ET LENTE ────────────────────────────────────────
        # Pas de pulse compliqué : juste une rotation très lente dans le bon sens.
        # La zone morte évite les oscillations près du centre.
        
        # ZONE MORTE ANTI-OSCILLATION : si erreur petite, robot immobile
        # On laisse 1.5x le seuil pour absorber l'inertie
        soft_zone = self.center_threshold * 1.5
        if abs(error_x) < soft_zone:
            self._stop()
            return

        # Vitesse de rotation FIXE et TRÈS LENTE
        # error>0 (cube à droite) → angular<0 → tourne droite
        rot_fixed = 0.15                        # rad/s, lent pour éviter inertie
        angular = -rot_fixed if error_x > 0 else rot_fixed

        # Rotation pure
        self._send_cmd(0.0, angular)

    # ── APPROCHER : vitesse fixe + correction proportionnelle simple ────────
    def _s_approcher_cube(self):
        """Approche du cube avec correction proportionnelle simple.
        
        Logique exacte du snippet qui a marché :
            erreur_x = cx - centre_img
            linear.x  = vitesse_robot_aller
            angular.z = -(gain_direction * erreur_x)
        
        Termine quand :
          - cube assez proche (area >= cube_min_area)  → APPROCHE_FINALE
          - cube perdu trop longtemps                  → CHERCHER
          - décentrage trop important                  → CENTRAGE (réalignement)
          - timeout                                    → CHERCHER
        """
        # Timeout
        if self._time_in_state() > self.to_approcher:
            self.get_logger().warn("Timeout APPROCHER_CUBE - retour CHERCHER")
            self._reset_cycle()
            self._change_state(State.CHERCHER_CUBE)
            return

        # Perte prolongée → retour à la recherche
        if self.cube_lost >= self.lost_frames:
            self.get_logger().warn("Cube perdu - retour CHERCHER")
            self._reset_cycle()
            self._change_state(State.CHERCHER_CUBE)
            return

        # Cube visible : vérifier proximité + correction angulaire
        if self.last_cube and self.last_cube.visible and self._is_cube_valid(self.last_cube):
            # Cube proche → APPROCHE_FINALE (aveugle 1.5s pour rentrer dans U)
            if self.last_cube.area >= self.cube_min_area:
                self._stop()
                self.get_logger().info(
                    f"Cube proche (area={self.last_cube.area:.0f}) → APPROCHE_FINALE"
                )
                self._change_state(State.APPROCHE_FINALE)
                return

            # ─── Calcul de l'erreur ────────────────────────────────────────
            centre_img = self.last_cube.img_width / 2.0
            if len(self.cube_cx_buffer) >= 2:
                avg_cx = sum(self.cube_cx_buffer) / len(self.cube_cx_buffer)
            else:
                avg_cx = self.last_cube.cx
            erreur_x = avg_cx - centre_img

            # ─── Garde-fou : si erreur trop grande, retour au CENTRAGE ────
            # (le cube a bougé ou on est trop décentré)
            recentrage_threshold = self.center_threshold * 3   # 3x le seuil
            if abs(erreur_x) > recentrage_threshold:
                self._stop()
                self.get_logger().warn(
                    f"Décentrage trop grand pendant approche (err={erreur_x:.0f}px) "
                    f"→ retour CENTRAGE_CUBE"
                )
                self._change_state(State.CENTRAGE_CUBE)
                return

            # ─── Logique simple : vitesse fixe + correction proportionnelle ──
            linear  = self.approach_speed
            angular = -(self.cube_gain * erreur_x)

            self._send_cmd(linear, angular)
        else:
            # Cube non visible cette frame : on continue d'avancer tout droit
            self._send_cmd(self.approach_speed, 0.0)

    # ── APPROCHE_FINALE : avance aveugle pour rentrer cube dans le U ────────
    def _s_approche_finale(self):
        """Avance tout droit (aveugle) pendant approach_blind_duration secondes
           pour faire entrer le cube dans le U du robot.
           
           Pas de centrage ni détection ici : le cube est forcément devant nous,
           on a juste besoin de finir d'avancer mécaniquement."""
        elapsed = self._time_in_state()

        if elapsed >= self.approach_blind_dur:
            self._stop()
            self.get_logger().info(
                f"Approche finale terminée ({elapsed:.1f}s) → SAISIR"
            )
            self._change_state(State.SAISIR)
            return

        # Avance tout droit, vitesse plus lente pour précision
        self._send_cmd(self.approach_speed * 0.7, 0.0)

    # ── SAISIR : servo déjà ouvert depuis détection, on ferme juste ─────────
    def _s_saisir(self):
        """Le servo a été ouvert dès la détection du cube (CHERCHER_CUBE).
           Le cube est maintenant dans le U après APPROCHE_FINALE.
           On ferme le servo et on attend qu'il soit fermé.
           
           Phases :
             1. FERMER       : envoie servo_close
             2. ATTENDRE_FERME : pause pour que le servo se ferme physiquement
             → CHERCHER_DEPOT
        """
        if self.saisir_phase == 'OUVRIR' or self.saisir_phase == 'ATTENDRE_OUVERT':
            # Premier passage dans SAISIR : on saute direct à FERMER
            # (le servo est déjà ouvert depuis CHERCHER_CUBE)
            self._stop()
            self._send_servo(self.servo_close)
            self.get_logger().info("Servo FERMÉ (cube saisi)")
            self.saisir_phase = 'ATTENDRE_FERME'
            self.action_start_time = time.time()
            return

        # Attente que le servo se ferme physiquement
        if self.saisir_phase == 'ATTENDRE_FERME':
            self._stop()
            if time.time() - self.action_start_time >= self.grab_duration:
                self.get_logger().info("Cube saisi → RETOUR_CENTRE")
                self._change_state(State.RETOUR_CENTRE)
            return

    def _s_retour_centre(self):
        """Marche arrière après SAISIR pour revenir vers le centre de l'arène.
           
           Permet au robot d'avoir de l'espace pour faire son 360° de
           recherche d'ArUco sans être bloqué par les bords de l'arène
           ou par le cube qu'il vient de saisir."""
        elapsed = self._time_in_state()
        
        if elapsed >= self.retour_centre_dur:
            self._stop()
            self.get_logger().info(
                f"Retour centre terminé ({elapsed:.1f}s) → CHERCHER_DEPOT"
            )
            self._change_state(State.CHERCHER_DEPOT)
            return
        
        # Marche arrière en ligne droite
        self._send_cmd(-self.backup_speed, 0.0)

    def _s_chercher_depot(self):
        if self._time_in_state() > self.to_chercher_depot:
            self.get_logger().warn("Timeout CHERCHER_DEPOT - abandon du cube")
            self._send_servo(self.servo_open)
            self._reset_cycle()
            self._change_state(State.CHERCHER_CUBE)
            return

        if (self.last_aruco and self._is_aruco_target(self.last_aruco)
                and self.aruco_stable >= self.stable_frames):
            self.get_logger().info(
                f"ArUco cible ID={self.target_aruco} détecté → CENTRAGE_DEPOT"
            )
            self._change_state(State.CENTRAGE_DEPOT)
            return

        rot_speed = self._search_rotation_cmd(self.search_rot_speed)
        self._send_cmd(0.0, rot_speed)

    def _s_centrage_depot(self):
        if self._time_in_state() > self.to_approcher * 2:
            self.get_logger().warn("Timeout CENTRAGE_DEPOT - retour CHERCHER_DEPOT")
            self._change_state(State.CHERCHER_DEPOT)
            return

        if self.aruco_lost >= self.lost_frames:
            self.get_logger().warn("ArUco perdu pendant centrage - retour CHERCHER_DEPOT")
            self._change_state(State.CHERCHER_DEPOT)
            return

        pulse_rot = 0.3
        pulse_pause = 0.8
        cycle_dur = pulse_rot + pulse_pause
        t_in_cycle = self._time_in_state() % cycle_dur

        if t_in_cycle >= pulse_rot:
            self._stop()
            if self.last_aruco and self.last_aruco.visible:
                if self.last_aruco.marker_id != self.target_aruco:
                    self.get_logger().warn(
                        f"Mauvais ArUco ({self.last_aruco.marker_id}) - "
                        f"attendu {self.target_aruco}"
                    )
                    self._change_state(State.CHERCHER_DEPOT)
                    return

                img_center = self.last_aruco.img_width / 2.0
                if len(self.aruco_cx_buffer) >= 2:
                    avg_cx = sum(self.aruco_cx_buffer) / len(self.aruco_cx_buffer)
                else:
                    avg_cx = self.last_aruco.cx
                error_x = avg_cx - img_center

                if abs(error_x) < self.center_threshold:
                    self._stop()
                    self.get_logger().info(
                        f"ArUco centré (err={error_x:.0f}px) → APPROCHER_DEPOT"
                    )
                    self._change_state(State.APPROCHER_DEPOT)
                    return

                self._centrage_depot_error = error_x
            return

        error_x = getattr(self, '_centrage_depot_error', None)
        if error_x is None:
            if self.last_aruco and self.last_aruco.visible:
                error_x = self.last_aruco.cx - self.last_aruco.img_width / 2.0
            else:
                self._stop()
                return

        rot_speed = 0.3 if error_x > 0 else -0.3
        self._send_cmd(0.0, -rot_speed)

    def _s_approcher_depot(self):
        if self._time_in_state() > self.to_approcher:
            self.get_logger().warn("Timeout APPROCHER_DEPOT - retour CHERCHER_DEPOT")
            self._change_state(State.CHERCHER_DEPOT)
            return

        if self.aruco_lost >= self.lost_frames:
            self.get_logger().warn("ArUco perdu pendant approche - retour CHERCHER_DEPOT")
            self._change_state(State.CHERCHER_DEPOT)
            return

        if self.last_aruco and self.last_aruco.visible:
            if self.last_aruco.marker_id != self.target_aruco:
                self.get_logger().warn(
                    f"Mauvais ArUco ({self.last_aruco.marker_id}) détecté - "
                    f"attendu {self.target_aruco} → CHERCHER_DEPOT"
                )
                self._change_state(State.CHERCHER_DEPOT)
                return

            if (self.last_aruco.distance_m > 0.01 and
                    self.last_aruco.distance_m <= self.deposit_stop_dist):
                self._stop()
                self.get_logger().info(
                    f"Distance dépôt atteinte ({self.last_aruco.distance_m:.2f}m) → DEPOSER"
                )
                self._change_state(State.DEPOSER)
                return

            img_center = self.last_aruco.img_width / 2.0
            if len(self.aruco_cx_buffer) >= 2:
                avg_cx = sum(self.aruco_cx_buffer) / len(self.aruco_cx_buffer)
            else:
                avg_cx = self.last_aruco.cx
            error_x = avg_cx - img_center

            if abs(error_x) < self.center_threshold:
                angular = 0.0
            else:
                angular = -self.aruco_gain * error_x

            self._send_cmd(self.approach_dep_speed, angular)
        else:
            self._send_cmd(self.approach_dep_speed, 0.0)

    def _s_deposer(self):
        if self._time_in_state() < 0.1:
            self._stop()
            self._send_servo(self.servo_open)
            self.action_start_time = time.time()
            x, y, _ = self._get_pose()
            if x is not None:
                self.backup_start_x = x
                self.backup_start_y = y
            return

        self._stop()
        if time.time() - self.action_start_time >= self.release_duration:
            self.get_logger().info("Cube déposé → RECULER")
            self._change_state(State.RECULER)

    def _s_reculer(self):
        if self._time_in_state() > self.to_approcher:
            self.get_logger().warn("Timeout RECULER - reset cycle")
            self._stop()
            self._end_cycle()
            return

        d = self._distance_from(self.backup_start_x, self.backup_start_y)
        if d >= self.backup_dist:
            self._stop()
            # Servo FERMÉ pour le prochain cycle (se ré-ouvrira dès détection cube)
            self._send_servo(self.servo_close)
            self.get_logger().info(f"Recul terminé ({d:.2f}m)")
            self._end_cycle()
            return

        self._send_cmd(-self.backup_speed, 0.0)

    def _s_fin(self):
        self._stop()

    def _s_fin_erreur(self):
        self._stop()

    # ══════════════════════════════════════════════════════════════════════
    # Cycles
    # ══════════════════════════════════════════════════════════════════════
    def _reset_cycle(self):
        self.locked_color = None
        self.target_aruco = -1

    def _end_cycle(self):
        self.cycles_done += 1
        self._reset_cycle()
        self.get_logger().info(
            f"═══ Cycle {self.cycles_done}/{self.max_cycles} terminé ═══"
        )

        if self.cycles_done >= self.max_cycles:
            self.get_logger().info("Nombre max de cycles atteint → FIN")
            self._change_state(State.FIN)
        else:
            self._change_state(State.CHERCHER_CUBE)

    def destroy_node(self):
        try:
            self._stop()
        except Exception:
            pass
        super().destroy_node()


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
