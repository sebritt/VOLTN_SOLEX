
from flask import Flask, jsonify, render_template_string
import serial.tools.list_ports
import serial
import threading

app = Flask(__name__)

latest_data = {
    "lat": None,
    "lon": None,
    "speed": 0,
    "formatted": "En attente de données GPS..."
}

history_speeds = []

HTML_PAGE = '''
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>GPS Live Map</title>
  <style>
    #map { height: 90vh; width: 100%; }
    #info {
      position: fixed;
      bottom: 10px;
      left: 10px;
      z-index: 9999;
      background-color: white;
      padding: 10px;
      border: 2px solid black;
      border-radius: 5px;
      font-family: monospace;
      white-space: pre-wrap;
      max-width: 40%;
    }
  </style>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
</head>
<body>
  <div id="map"></div>
  <div id="info">En attente de données GPS...</div>

  <script>
    const map = L.map('map').setView([48.0, -1.0], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 18,
    }).addTo(map);

    let currentMarker = null;
    let trail = [];

    function speedToColor(speed) {
      const ratio = Math.min(Math.max(speed / 120, 0), 1);
      const hue = (1 - ratio) * 240;
      return "hsl(" + hue + ", 100%, 50%)";
    }

    async function fetchData() {
      const res = await fetch('/data');
      const data = await res.json();
      document.getElementById('info').innerText = data.formatted;

      if (!data.lat || !data.lon) return;

      const lat = data.lat;
      const lon = data.lon;
      const speed = data.speed;
      const currentPoint = [lat, lon, speed];

      if (trail.length > 0) {
        const prev = trail[trail.length - 1];
        const segment = L.polyline([[prev[0], prev[1]], [lat, lon]], {
          color: speedToColor(prev[2]),
          weight: 3
        }).addTo(map);
      }

      trail.push(currentPoint);

      if (currentMarker) map.removeLayer(currentMarker);
      currentMarker = L.circleMarker([lat, lon], {
        radius: 6,
        color: 'red',
        fillColor: 'red',
        fillOpacity: 1.0
      }).addTo(map);

      map.panTo([lat, lon]);
    }

    setInterval(fetchData, 100);
  </script>
</body>
</html>
'''

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/data")
def data():
    return jsonify(latest_data)

def scan_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if "ACM" in p.device]

def read_serial(port):
    global latest_data, history_speeds
    ser = serial.Serial(port, 115200, timeout=1)
    headers = [
        "lat", "long", "height", "day", "month", "year", "hour", "minute", "second", "millis", "pdop", "speed_kmh",
        "ori_x", "ori_y", "ori_z",
        "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z",
        "mag_x", "mag_y", "mag_z",
        "lin_x", "lin_y", "lin_z",
        "grav_x", "grav_y", "grav_z",
        "temp", "cal_sys", "cal_gyro", "cal_accel", "cal_mag"
    ]
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(',')
            lat = float(parts[0])
            lon = float(parts[1])
            speed = float(parts[11])
            history_speeds.append(speed)
            vmax = max(history_speeds)
            vmean = sum(history_speeds) / len(history_speeds)
            formatted = ""
            for h, v in zip(headers, parts):
                formatted += f"{h:>10}: {v}\n"
            formatted += f"\n    Vitesse max: {vmax:.2f} km/h"
            formatted += f"\n Vitesse moyenne: {vmean:.2f} km/h"
            latest_data["lat"] = lat
            latest_data["lon"] = lon
            latest_data["speed"] = speed
            latest_data["formatted"] = formatted.strip()
        except Exception as e:
            print("Erreur:", e)

if __name__ == "__main__":
    ports = scan_ports()
    if not ports:
        print("Aucun port ACM trouvé.")
        exit(1)
    print("Ports disponibles :")
    for i, p in enumerate(ports):
        print(f"[{i}] {p}")
    idx = int(input("Choisis un port : "))
    port = ports[idx]
    print(f"Lecture sur {port}...")

    thread = threading.Thread(target=read_serial, args=(port,), daemon=True)
    thread.start()

    app.run(debug=False, port=5000)
