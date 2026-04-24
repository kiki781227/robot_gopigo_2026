"""
debug_server.py
───────────────
Serveur Flask minimal pour streamer l'image caméra (et plus tard les
annotations) dans un navigateur web.

Accessible sur : http://raspi-200:8080 (ou l'IP de la Pi)

Ce node s'abonne à /camera/image_raw et sert un flux MJPEG + une page HTML
simple avec des infos en temps réel.

Extension future : on ajoutera l'image annotée (/vision/debug_image),
la position odométrique, l'état de la mission, etc.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

try:
    from flask import Flask, Response, render_template_string
except ImportError:
    raise RuntimeError(
        "Flask n'est pas installé. Installe avec : "
        "pip3 install flask --break-system-packages"
    )


# ─── Page HTML servie à la racine ──────────────────────────────────────────
HTML_PAGE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <title>Robot GoPiGo3 - Debug</title>
    <style>
        body {
            background: #1e1e1e;
            color: #e0e0e0;
            font-family: -apple-system, BlinkMacSystemFont, sans-serif;
            margin: 0;
            padding: 20px;
        }
        h1 {
            color: #4a9eff;
            margin-bottom: 10px;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
        }
        .status {
            background: #2a2a2a;
            padding: 12px 16px;
            border-radius: 6px;
            margin-bottom: 16px;
            border-left: 3px solid #4a9eff;
        }
        .video-container {
            background: #000;
            border-radius: 6px;
            overflow: hidden;
            display: inline-block;
        }
        img {
            display: block;
            max-width: 100%;
            height: auto;
        }
        .label {
            color: #888;
            font-size: 0.9em;
        }
        .value {
            color: #e0e0e0;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot GoPiGo3 — Debug</h1>
        <div class="status">
            <span class="label">Flux :</span>
            <span class="value">/camera/image_raw</span>
            &nbsp;&nbsp;
            <span class="label">Status :</span>
            <span class="value">Live</span>
        </div>
        <div class="video-container">
            <img src="/video_feed" alt="Flux caméra"/>
        </div>
    </div>
</body>
</html>
"""


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
        self.latest_frame = None
        self.lock = threading.Lock()

        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # ── Flask ───────────────────────────────────────────────────────────
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)

        # Désactive les logs Flask par défaut (trop verbeux)
        import logging
        logging.getLogger('werkzeug').setLevel(logging.WARNING)

        self.flask_thread = threading.Thread(
            target=self.run_flask, daemon=True
        )
        self.flask_thread.start()

        self.get_logger().info(
            f"Debug server démarré sur http://{self.host}:{self.port}"
        )

    # ── ROS callback ───────────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image : {e}")

    # ── Flask routes ───────────────────────────────────────────────────────
    def index(self):
        return render_template_string(HTML_PAGE)

    def video_feed(self):
        return Response(
            self.generate_frames(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    def generate_frames(self):
        while True:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                # Pas encore de frame reçue
                import time
                time.sleep(0.05)
                continue

            # Encoder en JPEG
            ret, buffer = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if not ret:
                continue

            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def run_flask(self):
        # threaded=True pour servir plusieurs clients / endpoints en parallèle
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
