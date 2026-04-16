import cv2
import numpy as np
import threading
import time
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ─── PLAGES HSV PAR COULEUR ───────────────────────────────────────────────────
color_ranges = {
    'bleu':  {'h_min':100, 'h_max':130, 's_min':100, 's_max':255, 'v_min':50,  'v_max':255},
    'vert':  {'h_min':40,  'h_max':80,  's_min':60,  's_max':255, 'v_min':50,  'v_max':255},
    'rouge': {'h_min':0,   'h_max':10,  's_min':120, 's_max':255, 'v_min':70,  'v_max':255},
    'jaune': {'h_min':20,  'h_max':35,  's_min':100, 's_max':255, 'v_min':100, 'v_max':255},
}

color_bgr = {
    'bleu':  (255, 100,   0),
    'vert':  (  0, 200,   0),
    'rouge': (  0,   0, 220),
    'jaune': (  0, 200, 220),
}

# ─── CROP (% depuis le haut, 0-90) ───────────────────────────────────────────
crop_ratio = 0.40   # 35% — un peu moins de la moitié

# ─── FRAMES PARTAGÉES ─────────────────────────────────────────────────────────
frames = {
    'raw':         None,
    'mask_bleu':   None,
    'mask_vert':   None,
    'mask_rouge':  None,
    'mask_jaune':  None,
    'combined':    None,
}
frames_lock = threading.Lock()


# ─── BOUCLE CAMERA ────────────────────────────────────────────────────────────
def camera_loop():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Erreur : impossible d'ouvrir la camera")
        return

    print("Camera OK")
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # ── Crop ──────────────────────────────────────────────────────────────
        h, w        = frame.shape[:2]
        crop_start  = int(h * crop_ratio)
        frame       = frame[crop_start:, :]
        # ──────────────────────────────────────────────────────────────────────
        
        b, g, r = cv2.split(frame)
        g = cv2.subtract(g, 10)  # diminue le vert
        frame = cv2.merge([b, g, r])
        bilateral = cv2.bilateralFilter(frame, 5, 10, 75)
        hsv       = cv2.cvtColor(bilateral, cv2.COLOR_BGR2HSV)

        masks    = {}
        combined = frame.copy()

        for color_name, vals in color_ranges.items():
            lower = np.array([vals['h_min'], vals['s_min'], vals['v_min']])
            upper = np.array([vals['h_max'], vals['s_max'], vals['v_max']])
            mask  = cv2.inRange(hsv, lower, upper)

            # Rouge : double plage HSV
            if color_name == 'rouge':
                lower2 = np.array([170, vals['s_min'], vals['v_min']])
                upper2 = np.array([179, vals['s_max'], vals['v_max']])
                mask   = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))

            kernel = np.ones((5, 5), np.uint8)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            masks[color_name] = mask

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue
                cv2.drawContours(combined, [cnt], -1, color_bgr[color_name], 2)
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.putText(combined, color_name, (cx - 20, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                color_bgr[color_name], 2)

        def encode(img):
            _, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return jpg.tobytes()

        with frames_lock:
            frames['raw']        = encode(frame)
            frames['combined']   = encode(combined)
            frames['mask_bleu']  = encode(cv2.cvtColor(masks['bleu'],  cv2.COLOR_GRAY2BGR))
            frames['mask_vert']  = encode(cv2.cvtColor(masks['vert'],  cv2.COLOR_GRAY2BGR))
            frames['mask_rouge'] = encode(cv2.cvtColor(masks['rouge'], cv2.COLOR_GRAY2BGR))
            frames['mask_jaune'] = encode(cv2.cvtColor(masks['jaune'], cv2.COLOR_GRAY2BGR))

        time.sleep(1 / 20)

    cap.release()


# ─── GENERATEUR MJPEG ─────────────────────────────────────────────────────────
def mjpeg_generator(key, wfile):
    while True:
        with frames_lock:
            jpeg = frames.get(key)

        if jpeg is None:
            time.sleep(0.05)
            continue

        try:
            wfile.write(b'--frame\r\n')
            wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
            wfile.write(jpeg)
            wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            break

        time.sleep(1 / 15)


# ─── HANDLER HTTP ─────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path   = parsed.path
        params = parse_qs(parsed.query)

        valid_streams = ['/stream/raw', '/stream/combined',
                         '/stream/mask_bleu', '/stream/mask_vert',
                         '/stream/mask_rouge', '/stream/mask_jaune']

        if path in valid_streams:
            key = path.replace('/stream/', '')
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            mjpeg_generator(key, self.wfile)

        elif path == '/set':
            color = params.get('color', [None])[0]
            key   = params.get('key',   [None])[0]
            val   = params.get('val',   [None])[0]
            if color in color_ranges and key in color_ranges[color] and val:
                color_ranges[color][key] = int(val)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')

        elif path == '/set_crop':
            global crop_ratio
            val = params.get('val', [None])[0]
            if val:
                crop_ratio = float(val) / 100
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')

        elif path == '/save':
            print("\n=== VALEURS A COPIER DANS cube_detector_node.py ===")
            for color_name, vals in color_ranges.items():
                print(f"\n'{color_name}': (")
                print(f"    np.array([{vals['h_min']}, {vals['s_min']}, {vals['v_min']}]),  # lower")
                print(f"    np.array([{vals['h_max']}, {vals['s_max']}, {vals['v_max']}])   # upper")
                print(f"),")
            print(f"\ncrop_ratio = {crop_ratio:.2f}")
            print("====================================================\n")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"saved":true}')

        elif path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))

        else:
            self.send_response(404)
            self.end_headers()


# ─── PAGE HTML ────────────────────────────────────────────────────────────────
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Calibration HSV</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: monospace; background: #1a1a2e; color: #eee; padding: 20px; }
    h1 { color: #4fc3f7; margin-bottom: 16px; font-size: 20px; }

    .streams-top  { display: flex; gap: 10px; margin-bottom: 16px; flex-wrap: wrap; }
    .streams-masks { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }
    .stream-box { text-align: center; }
    .stream-box p { margin-bottom: 5px; font-size: 12px; }
    .p-bleu  { color: #64b5f6; }
    .p-vert  { color: #81c784; }
    .p-rouge { color: #e57373; }
    .p-jaune { color: #fff176; }
    .p-raw   { color: #aaa; }
    .p-comb  { color: #fff; font-weight: bold; }
    img.stream-lg { width: 360px; height: 270px; border: 2px solid #444; border-radius: 6px; display: block; }
    img.stream-sm { width: 240px; height: 180px; border: 2px solid #333; border-radius: 6px; display: block; }

    .tabs { display: flex; gap: 8px; margin-bottom: 16px; flex-wrap: wrap; }
    .tab {
      padding: 8px 18px; border: none; border-radius: 4px;
      cursor: pointer; font-family: monospace; font-weight: bold;
      font-size: 13px; opacity: 0.5; transition: opacity 0.2s;
    }
    .tab.active { opacity: 1; }
    .tab-bleu  { background: #1565c0; color: #fff; }
    .tab-vert  { background: #2e7d32; color: #fff; }
    .tab-rouge { background: #c62828; color: #fff; }
    .tab-jaune { background: #f9a825; color: #000; }

    .panel { background: #16213e; padding: 20px; border-radius: 8px; max-width: 500px; display: none; }
    .panel.active { display: block; }
    .row { display: grid; grid-template-columns: 60px 1fr 40px; align-items: center; gap: 10px; margin-bottom: 12px; }
    label { color: #4fc3f7; font-size: 13px; }
    input[type=range] { width: 100%; accent-color: #4fc3f7; }
    .val { color: #fff; text-align: right; font-size: 13px; }

    .crop-box { background: #16213e; padding: 20px; border-radius: 8px; max-width: 500px; margin-bottom: 16px; }
    .crop-box h2 { color: #4fc3f7; font-size: 14px; margin-bottom: 12px; }

    .btn-save { margin-top: 16px; padding: 10px 24px; background: #f57f17; color: #fff; border: none; border-radius: 4px; cursor: pointer; font-family: monospace; font-weight: bold; font-size: 13px; }
    .output { margin-top: 14px; background: #0a0a0a; padding: 12px; border-radius: 4px; color: #69ff47; font-size: 11px; display: none; white-space: pre; }
    .note { margin-top: 10px; font-size: 11px; color: #ff8a65; }
  </style>
</head>
<body>
  <h1>Calibration HSV — 4 couleurs simultanees</h1>

  <div class="streams-top">
    <div class="stream-box">
      <p class="p-raw">Camera brute (croppee)</p>
      <img class="stream-lg" src="/stream/raw">
    </div>
    <div class="stream-box">
      <p class="p-comb">Resultat combine (toutes couleurs)</p>
      <img class="stream-lg" src="/stream/combined">
    </div>
  </div>

  <div class="streams-masks">
    <div class="stream-box">
      <p class="p-bleu">Masque Bleu</p>
      <img class="stream-sm" src="/stream/mask_bleu">
    </div>
    <div class="stream-box">
      <p class="p-vert">Masque Vert</p>
      <img class="stream-sm" src="/stream/mask_vert">
    </div>
    <div class="stream-box">
      <p class="p-rouge">Masque Rouge</p>
      <img class="stream-sm" src="/stream/mask_rouge">
    </div>
    <div class="stream-box">
      <p class="p-jaune">Masque Jaune</p>
      <img class="stream-sm" src="/stream/mask_jaune">
    </div>
  </div>

  <!-- Slider crop -->
  <div class="crop-box">
    <h2>Crop — zone ignoree depuis le haut</h2>
    <div class="row">
      <label>Crop</label>
      <input type="range" min="0" max="90" value="35" oninput="setCrop(this.value)">
      <span class="val" id="d-crop">35%</span>
    </div>
  </div>

  <div class="tabs">
    <button class="tab tab-bleu active" onclick="selectColor('bleu')">Bleu</button>
    <button class="tab tab-vert"        onclick="selectColor('vert')">Vert</button>
    <button class="tab tab-rouge"       onclick="selectColor('rouge')">Rouge</button>
    <button class="tab tab-jaune"       onclick="selectColor('jaune')">Jaune</button>
  </div>

  <div id="panel-bleu" class="panel active">
    <div class="row"><label>H min</label><input type="range" id="bleu-h_min" min="0" max="179" value="100" oninput="upd('bleu','h_min',this.value)"><span class="val" id="d-bleu-h_min">100</span></div>
    <div class="row"><label>H max</label><input type="range" id="bleu-h_max" min="0" max="179" value="130" oninput="upd('bleu','h_max',this.value)"><span class="val" id="d-bleu-h_max">130</span></div>
    <div class="row"><label>S min</label><input type="range" id="bleu-s_min" min="0" max="255" value="100" oninput="upd('bleu','s_min',this.value)"><span class="val" id="d-bleu-s_min">100</span></div>
    <div class="row"><label>S max</label><input type="range" id="bleu-s_max" min="0" max="255" value="255" oninput="upd('bleu','s_max',this.value)"><span class="val" id="d-bleu-s_max">255</span></div>
    <div class="row"><label>V min</label><input type="range" id="bleu-v_min" min="0" max="255" value="50"  oninput="upd('bleu','v_min',this.value)"><span class="val" id="d-bleu-v_min">50</span></div>
    <div class="row"><label>V max</label><input type="range" id="bleu-v_max" min="0" max="255" value="255" oninput="upd('bleu','v_max',this.value)"><span class="val" id="d-bleu-v_max">255</span></div>
  </div>

  <div id="panel-vert" class="panel">
    <div class="row"><label>H min</label><input type="range" id="vert-h_min" min="0" max="179" value="40" oninput="upd('vert','h_min',this.value)"><span class="val" id="d-vert-h_min">40</span></div>
    <div class="row"><label>H max</label><input type="range" id="vert-h_max" min="0" max="179" value="80" oninput="upd('vert','h_max',this.value)"><span class="val" id="d-vert-h_max">80</span></div>
    <div class="row"><label>S min</label><input type="range" id="vert-s_min" min="0" max="255" value="60" oninput="upd('vert','s_min',this.value)"><span class="val" id="d-vert-s_min">60</span></div>
    <div class="row"><label>S max</label><input type="range" id="vert-s_max" min="0" max="255" value="255" oninput="upd('vert','s_max',this.value)"><span class="val" id="d-vert-s_max">255</span></div>
    <div class="row"><label>V min</label><input type="range" id="vert-v_min" min="0" max="255" value="50" oninput="upd('vert','v_min',this.value)"><span class="val" id="d-vert-v_min">50</span></div>
    <div class="row"><label>V max</label><input type="range" id="vert-v_max" min="0" max="255" value="255" oninput="upd('vert','v_max',this.value)"><span class="val" id="d-vert-v_max">255</span></div>
  </div>

  <div id="panel-rouge" class="panel">
    <p class="note">Rouge detecte automatiquement sur deux plages HSV (0-10 et 170-179)</p><br>
    <div class="row"><label>H min</label><input type="range" id="rouge-h_min" min="0" max="179" value="0"   oninput="upd('rouge','h_min',this.value)"><span class="val" id="d-rouge-h_min">0</span></div>
    <div class="row"><label>H max</label><input type="range" id="rouge-h_max" min="0" max="179" value="10"  oninput="upd('rouge','h_max',this.value)"><span class="val" id="d-rouge-h_max">10</span></div>
    <div class="row"><label>S min</label><input type="range" id="rouge-s_min" min="0" max="255" value="120" oninput="upd('rouge','s_min',this.value)"><span class="val" id="d-rouge-s_min">120</span></div>
    <div class="row"><label>S max</label><input type="range" id="rouge-s_max" min="0" max="255" value="255" oninput="upd('rouge','s_max',this.value)"><span class="val" id="d-rouge-s_max">255</span></div>
    <div class="row"><label>V min</label><input type="range" id="rouge-v_min" min="0" max="255" value="70"  oninput="upd('rouge','v_min',this.value)"><span class="val" id="d-rouge-v_min">70</span></div>
    <div class="row"><label>V max</label><input type="range" id="rouge-v_max" min="0" max="255" value="255" oninput="upd('rouge','v_max',this.value)"><span class="val" id="d-rouge-v_max">255</span></div>
  </div>

  <div id="panel-jaune" class="panel">
    <div class="row"><label>H min</label><input type="range" id="jaune-h_min" min="0" max="179" value="20"  oninput="upd('jaune','h_min',this.value)"><span class="val" id="d-jaune-h_min">20</span></div>
    <div class="row"><label>H max</label><input type="range" id="jaune-h_max" min="0" max="179" value="35"  oninput="upd('jaune','h_max',this.value)"><span class="val" id="d-jaune-h_max">35</span></div>
    <div class="row"><label>S min</label><input type="range" id="jaune-s_min" min="0" max="255" value="100" oninput="upd('jaune','s_min',this.value)"><span class="val" id="d-jaune-s_min">100</span></div>
    <div class="row"><label>S max</label><input type="range" id="jaune-s_max" min="0" max="255" value="255" oninput="upd('jaune','s_max',this.value)"><span class="val" id="d-jaune-s_max">255</span></div>
    <div class="row"><label>V min</label><input type="range" id="jaune-v_min" min="0" max="255" value="100" oninput="upd('jaune','v_min',this.value)"><span class="val" id="d-jaune-v_min">100</span></div>
    <div class="row"><label>V max</label><input type="range" id="jaune-v_max" min="0" max="255" value="255" oninput="upd('jaune','v_max',this.value)"><span class="val" id="d-jaune-v_max">255</span></div>
  </div>

  <button class="btn-save" onclick="save()">Sauvegarder toutes les couleurs</button>
  <div class="output" id="out"></div>

  <script>
    function selectColor(name) {
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.querySelector('.tab-' + name).classList.add('active');
      document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
      document.getElementById('panel-' + name).classList.add('active');
    }

    function upd(color, key, val) {
      document.getElementById('d-' + color + '-' + key).textContent = val;
      fetch('/set?color=' + color + '&key=' + key + '&val=' + val);
    }

    function setCrop(val) {
      document.getElementById('d-crop').textContent = val + '%';
      fetch('/set_crop?val=' + val);
    }

    function save() {
      fetch('/save');
      const colors = ['bleu','vert','rouge','jaune'];
      const keys   = ['h_min','h_max','s_min','s_max','v_min','v_max'];
      let txt = '';
      colors.forEach(c => {
        const v = {};
        keys.forEach(k => { v[k] = document.getElementById(c+'-'+k).value; });
        txt += "'" + c + "': (\\n";
        txt += "    np.array([" + v.h_min + ", " + v.s_min + ", " + v.v_min + "]),  # lower\\n";
        txt += "    np.array([" + v.h_max + ", " + v.s_max + ", " + v.v_max + "])   # upper\\n";
        txt += "),\\n";
      });
      const cropVal = document.querySelector('.crop-box input').value;
      txt += "\\ncrop_ratio = " + (cropVal / 100).toFixed(2);
      const out = document.getElementById('out');
      out.style.display = 'block';
      out.textContent = txt;
    }
  </script>
</body>
</html>
"""

# ─── MAIN ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Demarrage camera...")
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()

    print("Attente premiere frame...")
    while frames['raw'] is None:
        time.sleep(0.1)
    print("Premiere frame OK !")

    port = 8080
    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    print(f"Serveur demarre -> http://TON_IP_PI:{port}")
    print("Ctrl+C pour arreter")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nArret")
