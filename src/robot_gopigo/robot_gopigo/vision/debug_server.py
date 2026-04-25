"""
debug_server.py
───────────────
Serveur Flask qui sert :
  - Une page HTML de debug
  - Un flux MJPEG annoté (ArUco + cubes dessinés sur l'image)
  - Un endpoint JSON /status avec l'état courant (odo, détections, mission)

Accessible sur : http://raspi-200:8080

S'abonne aux topics :
  - /camera/image_raw  : flux caméra
  - /aruco_detect      : détections ArUco (pour dessin + panneau)
  - /cube_detect       : détections cubes (pour dessin + panneau)
  - /odom              : position du robot (pour panneau)
  - /mission_state     : état de la mission (publié par mission_node)

Publie :
  - /cmd_vel (bouton stop d'urgence depuis la page web)
"""

import json
import math
import threading
import logging

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from robot_gopigo_msgs.msg import ArucoDetect, CubeDetect

try:
    from flask import Flask, Response, jsonify, request
except ImportError:
    raise RuntimeError(
        "Flask n'est pas installé : pip3 install flask --break-system-packages"
    )


# ═══════════════════════════════════════════════════════════════════════════
# Page HTML
# ═══════════════════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<title>Robot GoPiGo3 — Debug</title>
<style>
  * { box-sizing: border-box; }
  body {
    background: #1a1a1a;
    color: #e0e0e0;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    margin: 0;
    padding: 20px;
  }
  h1 {
    color: #4a9eff;
    margin: 0 0 20px 0;
    font-size: 1.6em;
  }
  .container {
    display: grid;
    grid-template-columns: 1fr 340px;
    gap: 20px;
    max-width: 1400px;
    margin: 0 auto;
  }
  @media (max-width: 900px) {
    .container { grid-template-columns: 1fr; }
  }
  .video-panel {
    background: #000;
    border-radius: 8px;
    overflow: hidden;
    border: 1px solid #333;
  }
  .video-panel img {
    display: block;
    width: 100%;
    height: auto;
  }
  .info-panel {
    display: flex;
    flex-direction: column;
    gap: 14px;
  }
  .card {
    background: #252525;
    border-radius: 8px;
    padding: 14px 16px;
    border-left: 3px solid #4a9eff;
  }
  .card.mission { border-left-color: #ffa94a; }
  .card.cube    { border-left-color: #4aff8a; }
  .card.aruco   { border-left-color: #ff4a9e; }
  .card.odom    { border-left-color: #a94aff; }
  .card-title {
    font-size: 0.75em;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    margin-bottom: 8px;
  }
  .row {
    display: flex;
    justify-content: space-between;
    padding: 3px 0;
    font-size: 0.92em;
  }
  .row .label { color: #aaa; }
  .row .value { color: #e0e0e0; font-weight: 600; font-family: 'Menlo', monospace; }
  .state-big {
    font-size: 1.3em;
    font-weight: 700;
    color: #ffa94a;
    margin: 4px 0 8px 0;
    font-family: 'Menlo', monospace;
  }
  .badge-ok  { color: #4aff8a; }
  .badge-no  { color: #666; }
  button.estop {
    background: #ff3d3d;
    color: white;
    border: none;
    padding: 14px;
    font-size: 1em;
    font-weight: 700;
    border-radius: 8px;
    cursor: pointer;
    letter-spacing: 0.05em;
  }
  button.estop:hover { background: #ff1d1d; }
  button.estop:active { transform: scale(0.98); }
</style>
</head>
<body>
<h1>🤖 Robot GoPiGo3 — Debug</h1>

<div class="container">
  <div class="video-panel">
    <img src="/video_feed" alt="Flux vidéo annoté"/>
  </div>

  <div class="info-panel">
    <div class="card mission">
      <div class="card-title">État Mission</div>
      <div class="state-big" id="state">—</div>
      <div class="row"><span class="label">Cycle</span><span class="value" id="cycle">—</span></div>
      <div class="row"><span class="label">Cible couleur</span><span class="value" id="target-color">—</span></div>
      <div class="row"><span class="label">Cible ArUco</span><span class="value" id="target-aruco">—</span></div>
    </div>

    <div class="card odom">
      <div class="card-title">Odométrie</div>
      <div class="row"><span class="label">x</span><span class="value" id="odom-x">—</span></div>
      <div class="row"><span class="label">y</span><span class="value" id="odom-y">—</span></div>
      <div class="row"><span class="label">θ</span><span class="value" id="odom-theta">—</span></div>
    </div>

    <div class="card cube">
      <div class="card-title">Cube détecté</div>
      <div class="row"><span class="label">Visible</span><span class="value" id="cube-vis">—</span></div>
      <div class="row"><span class="label">Couleur</span><span class="value" id="cube-color">—</span></div>
      <div class="row"><span class="label">Position px</span><span class="value" id="cube-pos">—</span></div>
      <div class="row"><span class="label">Aire</span><span class="value" id="cube-area">—</span></div>
    </div>

    <div class="card aruco">
      <div class="card-title">ArUco détecté</div>
      <div class="row"><span class="label">Visible</span><span class="value" id="aruco-vis">—</span></div>
      <div class="row"><span class="label">ID</span><span class="value" id="aruco-id">—</span></div>
      <div class="row"><span class="label">Distance</span><span class="value" id="aruco-dist">—</span></div>
      <div class="row"><span class="label">Bearing</span><span class="value" id="aruco-bearing">—</span></div>
    </div>

    <button class="estop" onclick="estop()">⛔ STOP D'URGENCE</button>
  </div>
</div>

<script>
function fmt(v, digits=2) {
  if (v === null || v === undefined) return '—';
  return Number(v).toFixed(digits);
}

async function refresh() {
  try {
    const r = await fetch('/status');
    const d = await r.json();

    document.getElementById('state').textContent = d.mission_state || '—';
    document.getElementById('cycle').textContent =
      (d.cycles_done ?? '?') + ' / ' + (d.max_cycles ?? '?');
    document.getElementById('target-color').textContent = d.locked_color || '—';
    document.getElementById('target-aruco').textContent =
      (d.target_aruco_id === -1 || d.target_aruco_id === null) ? '—' : d.target_aruco_id;

    document.getElementById('odom-x').textContent = fmt(d.odom_x) + ' m';
    document.getElementById('odom-y').textContent = fmt(d.odom_y) + ' m';
    document.getElementById('odom-theta').textContent = fmt(d.odom_theta_deg, 1) + '°';

    const cv = d.cube_visible;
    document.getElementById('cube-vis').innerHTML =
      cv ? '<span class="badge-ok">OUI</span>' : '<span class="badge-no">non</span>';
    document.getElementById('cube-color').textContent = d.cube_color || '—';
    document.getElementById('cube-pos').textContent =
      cv ? `(${d.cube_cx}, ${d.cube_cy})` : '—';
    document.getElementById('cube-area').textContent =
      cv ? fmt(d.cube_area, 0) + ' px²' : '—';

    const av = d.aruco_visible;
    document.getElementById('aruco-vis').innerHTML =
      av ? '<span class="badge-ok">OUI</span>' : '<span class="badge-no">non</span>';
    document.getElementById('aruco-id').textContent = av ? d.aruco_id : '—';
    document.getElementById('aruco-dist').textContent =
      av ? fmt(d.aruco_distance_m) + ' m' : '—';
    document.getElementById('aruco-bearing').textContent =
      av ? fmt(d.aruco_bearing_deg, 1) + '°' : '—';
  } catch (e) {
    console.error(e);
  }
}

async function estop() {
  await fetch('/estop', { method: 'POST' });
}

setInterval(refresh, 400);
refresh();
</script>
</body>
</html>
"""


# ═══════════════════════════════════════════════════════════════════════════
# Couleurs BGR pour dessin (OpenCV)
# ═══════════════════════════════════════════════════════════════════════════
COLOR_BGR = {
    'bleu':  (255, 140, 40),
    'vert':  (40, 220, 40),
    'rouge': (40, 40, 230),
    'jaune': (40, 220, 220),
}


class DebugServerNode(Node):

    def __init__(self):
        super().__init__('debug_server')

        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('jpeg_quality', 70)

        self.port         = self.get_parameter('port').value
        self.host         = self.get_parameter('host').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # ── ROS ─────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # État partagé
        self.latest_frame = None      # image OpenCV BGR
        self.latest_aruco = None      # ArucoDetect
        self.latest_cube  = None      # CubeDetect
        self.latest_odom  = None      # Odometry
        self.latest_mission_state = "—"

        self.create_subscription(Image, '/camera/image_raw', self._img_cb, 10)
        self.create_subscription(ArucoDetect, '/aruco_detect', self._aruco_cb, 10)
        self.create_subscription(CubeDetect, '/cube_detect', self._cube_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/mission_state', self._mission_cb, 10)

        # Stop d'urgence
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Flask ───────────────────────────────────────────────────────────
        self.app = Flask(__name__)
        self.app.add_url_rule('/',           'index',      self.index)
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/status',     'status',     self.status)
        self.app.add_url_rule('/estop',      'estop',      self.estop, methods=['POST'])

        logging.getLogger('werkzeug').setLevel(logging.WARNING)

        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        self.get_logger().info(
            f"Debug server démarré sur http://{self.host}:{self.port}"
        )

    # ── Callbacks ROS ───────────────────────────────────────────────────────
    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image: {e}")

    def _aruco_cb(self, msg):
        self.latest_aruco = msg

    def _cube_cb(self, msg):
        self.latest_cube = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _mission_cb(self, msg):
        self.latest_mission_state = msg.data

    # ── Flask routes ────────────────────────────────────────────────────────
    def index(self):
        return HTML_PAGE

    def video_feed(self):
        return Response(
            self.generate_frames(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    def status(self):
        data = self._build_status_dict()
        return jsonify(data)

    def estop(self):
        # Envoie plusieurs zéros pour être sûr
        for _ in range(5):
            msg = Twist()
            self.pub_cmd.publish(msg)
        self.get_logger().warn("STOP D'URGENCE demandé depuis l'interface")
        return ('', 204)

    # ── Génération frames annotées ──────────────────────────────────────────
    def _annotate(self, frame):
        """Dessine ArUco + cube détectés par-dessus l'image."""
        annotated = frame.copy()
        h, w = annotated.shape[:2]

        # ArUco
        aruco = self.latest_aruco
        if aruco is not None and aruco.visible:
            cx, cy = int(aruco.cx), int(aruco.cy)
            sz = int(max(aruco.size, 10))
            # Rectangle autour
            cv2.rectangle(annotated,
                          (cx - sz // 2, cy - sz // 2),
                          (cx + sz // 2, cy + sz // 2),
                          (255, 80, 200), 2)
            # Texte ID + distance
            label = f"ID={aruco.marker_id} d={aruco.distance_m:.2f}m"
            cv2.putText(annotated, label, (cx - sz // 2, cy - sz // 2 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 200), 2)

        # Cube
        cube = self.latest_cube
        if cube is not None and cube.visible:
            cx, cy = int(cube.cx), int(cube.cy)
            color_bgr = COLOR_BGR.get(cube.color, (255, 255, 255))
            radius = max(int(math.sqrt(max(cube.area, 1)) / 2), 8)
            cv2.circle(annotated, (cx, cy), radius, color_bgr, 2)
            label = f"{cube.color} ({int(cube.area)})"
            cv2.putText(annotated, label, (cx - radius, cy - radius - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        # Cross au centre image (repère)
        cv2.line(annotated, (w // 2 - 10, h // 2), (w // 2 + 10, h // 2),
                 (100, 100, 100), 1)
        cv2.line(annotated, (w // 2, h // 2 - 10), (w // 2, h // 2 + 10),
                 (100, 100, 100), 1)

        # Bannière état mission en haut
        state_txt = f"STATE: {self.latest_mission_state}"
        cv2.rectangle(annotated, (0, 0), (w, 28), (30, 30, 30), -1)
        cv2.putText(annotated, state_txt, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        return annotated

    def generate_frames(self):
        import time as _t
        while True:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                _t.sleep(0.05)
                continue

            annotated = self._annotate(frame)

            ret, buffer = cv2.imencode(
                '.jpg', annotated,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if not ret:
                continue

            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    # ── Build JSON status ───────────────────────────────────────────────────
    def _build_status_dict(self):
        d = {
            'mission_state': self.latest_mission_state,
            'cycles_done':   None,
            'max_cycles':    None,
            'locked_color':  None,
            'target_aruco_id': None,
            'odom_x':        None,
            'odom_y':        None,
            'odom_theta_deg': None,
            'cube_visible':  False,
            'cube_color':    None,
            'cube_cx':       None,
            'cube_cy':       None,
            'cube_area':     None,
            'aruco_visible':    False,
            'aruco_id':         None,
            'aruco_distance_m': None,
            'aruco_bearing_deg': None,
        }

        # Odométrie
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            q = self.latest_odom.pose.pose.orientation
            theta = 2.0 * math.atan2(q.z, q.w)
            d['odom_x'] = p.x
            d['odom_y'] = p.y
            d['odom_theta_deg'] = math.degrees(theta)

        # Cube
        if self.latest_cube is not None:
            d['cube_visible'] = self.latest_cube.visible
            if self.latest_cube.visible:
                d['cube_color'] = self.latest_cube.color
                d['cube_cx']    = self.latest_cube.cx
                d['cube_cy']    = self.latest_cube.cy
                d['cube_area']  = self.latest_cube.area

        # ArUco
        if self.latest_aruco is not None:
            d['aruco_visible'] = self.latest_aruco.visible
            if self.latest_aruco.visible:
                d['aruco_id']         = self.latest_aruco.marker_id
                d['aruco_distance_m'] = self.latest_aruco.distance_m
                d['aruco_bearing_deg'] = math.degrees(self.latest_aruco.bearing_rad)

        # État mission décomposé depuis le String publié
        # Format attendu (optionnel) : "STATE|cycle=2/10|color=bleu|aruco=2"
        raw = self.latest_mission_state
        if raw and '|' in raw:
            parts = raw.split('|')
            d['mission_state'] = parts[0]
            for p in parts[1:]:
                if '=' in p:
                    k, v = p.split('=', 1)
                    k = k.strip()
                    v = v.strip()
                    if k == 'cycle' and '/' in v:
                        done, total = v.split('/')
                        try:
                            d['cycles_done'] = int(done)
                            d['max_cycles']  = int(total)
                        except ValueError:
                            pass
                    elif k == 'color':
                        d['locked_color'] = v if v != '-' else None
                    elif k == 'aruco':
                        try:
                            d['target_aruco_id'] = int(v)
                        except ValueError:
                            pass
        else:
            d['mission_state'] = raw or '—'

        return d

    def run_flask(self):
        self.app.run(
            host=self.host,
            port=self.port,
            threaded=True,
            use_reloader=False
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebugServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
