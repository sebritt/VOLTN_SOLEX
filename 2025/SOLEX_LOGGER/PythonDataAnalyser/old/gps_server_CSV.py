from flask import Flask, jsonify, render_template_string, request
import csv
import os

app = Flask(__name__)
all_points = []

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset=\"utf-8\" />
  <title>GPS Visualizer Optimisé</title>
  <style>
    html, body {
      margin: 0;
      padding: 0;
      height: 100%;
    }
    #map {
      height: 100vh;
      width: 100%;
    }
    #info, #controls {
      position: fixed;
      z-index: 9999;
      background-color: white;
      padding: 10px;
      border: 2px solid black;
      border-radius: 5px;
      font-family: monospace;
      white-space: pre-wrap;
      max-width: 40%;
    }
    #info {
      top: 10px;
      left: 10px;
    }
    #controls {
      bottom: 10px;
      left: 10px;
      width: 300px;
    }
    input[type=range] {
      width: 100%;
    }
  </style>
  <link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet/dist/leaflet.css\" />
  <script src=\"https://unpkg.com/leaflet/dist/leaflet.js\"></script>
</head>
<body>
  <div id=\"map\"></div>
  <div id=\"info\">En attente de données GPS...</div>
  <div id=\"controls\">
    <label for=\"timeRange\">Temps: <span id=\"timeValue\">00:00</span></label><br>
    <input type=\"range\" id=\"timeRange\" min=\"0\" max=\"0\" value=\"0\" step=\"1\">
  </div>

  <script>
    const map = L.map('map').setView([48.0, -1.0], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 18,
    }).addTo(map);

    let currentMarker = null;
    let trail = [];
    let polylines = [];
    let allData = [];
    let lastIndex = -1;

    function speedToColor(speed) {
      const ratio = Math.min(Math.max(speed / 120, 0), 1);
      const hue = (1 - ratio) * 240;
      return "hsl(" + hue + ", 100%, 50%)";
    }

    function formatTime(ms) {
      const totalSec = Math.floor(ms / 1000);
      const hrs = Math.floor(totalSec / 3600);
      const mins = Math.floor((totalSec % 3600) / 60);
      const secs = totalSec % 60;
      return (hrs > 0 ? String(hrs).padStart(2, '0') + ':' : '') +
             String(mins).padStart(2, '0') + ':' + String(secs).padStart(2, '0');
    }

    function drawUpTo(index) {
      if (index === lastIndex || allData.length === 0) return;

      if (index > lastIndex) {
        for (let i = lastIndex + 1; i <= index; i++) {
          const pt = allData[i];
          trail.push([pt.lat, pt.lon, pt.speed]);

          if (trail.length > 1) {
            const prev = trail[trail.length - 2];
            const seg = L.polyline([[prev[0], prev[1]], [pt.lat, pt.lon]], {
              color: speedToColor(prev[2]),
              weight: 3
            }).addTo(map);
            polylines.push(seg);
          }
        }
      } else {
        for (let i = lastIndex; i > index; i--) {
          const seg = polylines.pop();
          if (seg) map.removeLayer(seg);
          trail.pop();
        }
      }

      if (currentMarker) map.removeLayer(currentMarker);
      const last = allData[index];
      currentMarker = L.circleMarker([last.lat, last.lon], {
        radius: 6,
        color: 'red',
        fillColor: 'red',
        fillOpacity: 1.0
      }).addTo(map);

      map.panTo([last.lat, last.lon]);
      document.getElementById('info').innerText = last.formatted;
      document.getElementById('timeValue').innerText = formatTime(last.millis);
      lastIndex = index;
    }

    async function init() {
      const res = await fetch('/data?mode=all');
      allData = await res.json();

      const slider = document.getElementById("timeRange");
      slider.max = allData.length - 1;
      slider.value = allData.length - 1;

      slider.addEventListener("input", e => {
        const index = parseInt(e.target.value);
        drawUpTo(index);
      });

      drawUpTo(allData.length - 1);
    }

    init();
  </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/data")
def get_data():
    mode = request.args.get("mode")
    if mode == "all":
        return jsonify(all_points)
    return jsonify([])

def load_csv(path):
    global all_points
    headers = [
        "lat", "long", "height", "day", "month", "year", "hour", "minute", "second", "millis", "pdop", "speed_kmh",
        "ori_x", "ori_y", "ori_z", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
        "mag_x", "mag_y", "mag_z", "lin_x", "lin_y", "lin_z", "grav_x", "grav_y", "grav_z",
        "temp", "cal_sys", "cal_gyro", "cal_accel", "cal_mag"
    ]
    history_speeds = []

    with open(path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader, None)
        for parts in reader:
            try:
                lat = float(parts[0])
                lon = float(parts[1])
                millis = int(parts[9])
                speed = float(parts[11])
                history_speeds.append(speed)
                vmax = max(history_speeds)
                vmean = sum(history_speeds) / len(history_speeds)
                formatted = ""
                for h, v in zip(headers, parts):
                    formatted += f"{h:>10}: {v}\n"
                formatted += f"\n    Vitesse max: {vmax:.2f} km/h"
                formatted += f"\n Vitesse moyenne: {vmean:.2f} km/h"
                all_points.append({
                    "lat": lat,
                    "lon": lon,
                    "speed": speed,
                    "millis": millis,
                    "formatted": formatted.strip()
                })
            except:
                continue

if __name__ == "__main__":
    path = input("Entrez le chemin absolu du fichier CSV : ").strip()
    if not os.path.isfile(path):
        print("Fichier introuvable.")
        exit(1)

    load_csv(path)
    print(f"{len(all_points)} points chargés.")
    app.run(debug=False, port=5000)
