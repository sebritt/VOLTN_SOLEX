from flask import Flask, jsonify, render_template_string
import serial.tools.list_ports
import serial
import threading
import csv
import os
import platform

app = Flask(__name__)

latest_data = {
    "lat": None,
    "lon": None,
    "speed": 0,
    "formatted": "En attente de données GPS..."
}

history_speeds = []
data_trail = []
mode = None

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>GPS CSV Map</title>
  <style>
    #map { height: 90vh; width: 100%; }
    #info, #controls {
      position: fixed;
      z-index: 9999;
      background-color: white;
      padding: 10px;
      border: 2px solid black;
      border-radius: 5px;
      font-family: monospace;
      white-space: pre-wrap;
    }
    #info {
      bottom: 10px;
      left: 10px;
      max-width: 40%;
    }
    #controls {
      bottom: 10px;
      right: 10px;
      text-align: center;
    }
    #slider {
      width: 100%;
    }
    button {
      margin-top: 5px;
      padding: 4px 10px;
    }
  </style>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
</head>
<body>
  <div id="map"></div>
  <div id="info">En attente de données GPS...</div>
  <div id="controls">
    <input type="range" id="slider" min="0" value="0">
    <br>
    <button id="play">Lecture</button>
    <button id="pause">Pause</button>
    <button id="replay">Rejouer</button>
  </div>
  <script>
    const map = L.map('map').setView([48.0, -1.0], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 18,
    }).addTo(map);

    let currentMarker = null;
    let trail = [];
    let segments = [];
    let playing = false;
    let currentIndex = 0;
    let interval = null;

    const slider = document.getElementById("slider");
    const playBtn = document.getElementById("play");
    const pauseBtn = document.getElementById("pause");
    const replayBtn = document.getElementById("replay");

    function speedToColor(speed) {
      const ratio = Math.min(Math.max(speed / 120, 0), 1);
      const hue = (1 - ratio) * 240;
      return "hsl(" + hue + ", 100%, 50%)";
    }

    function updateMap(index) {
      if (index < 0 || index >= trail.length) return;
      currentIndex = index;
      const point = trail[index];
      document.getElementById('info').innerText = point.formatted;

      if (currentMarker) map.removeLayer(currentMarker);
      currentMarker = L.circleMarker([point.lat, point.lon], {
        radius: 6,
        color: 'red',
        fillColor: 'red',
        fillOpacity: 1.0
      }).addTo(map);
      map.panTo([point.lat, point.lon]);

      for (let seg of segments) map.removeLayer(seg);
      segments = [];
      for (let i = 1; i <= index; i++) {
        const prev = trail[i - 1];
        const curr = trail[i];
        const seg = L.polyline([[prev.lat, prev.lon], [curr.lat, curr.lon]], {
          color: speedToColor(prev.speed),
          weight: 3
        }).addTo(map);
        segments.push(seg);
      }

      slider.value = index;
    }

    function startPlayback() {
      if (interval) clearInterval(interval);
      playing = true;
      interval = setInterval(() => {
        if (currentIndex < trail.length - 1) {
          updateMap(currentIndex + 1);
        } else {
          clearInterval(interval);
        }
      }, 100);
    }

    function stopPlayback() {
      if (interval) clearInterval(interval);
      playing = false;
    }

    function resetPlayback() {
      stopPlayback();
      updateMap(0);
    }

    slider.addEventListener("input", () => {
      stopPlayback();
      updateMap(parseInt(slider.value));
    });

    playBtn.addEventListener("click", startPlayback);
    pauseBtn.addEventListener("click", stopPlayback);
    replayBtn.addEventListener("click", resetPlayback);

    async function initCSVMode() {
      const res = await fetch('/data_csv');
      trail = await res.json();
      if (trail.length === 0) return;
      slider.max = trail.length - 1;
      updateMap(0);
    }

    async function initLiveMode() {
      document.getElementById('controls').style.display = "none";
      setInterval(async () => {
        const res = await fetch('/data');
        const data = await res.json();
        document.getElementById('info').innerText = data.formatted;
        if (!data.lat || !data.lon) return;
        const lat = data.lat;
        const lon = data.lon;
        const speed = data.speed;
        if (trail.length > 0) {
          const prev = trail[trail.length - 1];
          const segment = L.polyline([[prev[0], prev[1]], [lat, lon]], {
            color: speedToColor(prev[2]),
            weight: 3
          }).addTo(map);
        }
        trail.push([lat, lon, speed]);
        if (currentMarker) map.removeLayer(currentMarker);
        currentMarker = L.circleMarker([lat, lon], {
          radius: 6,
          color: 'red',
          fillColor: 'red',
          fillOpacity: 1.0
        }).addTo(map);
        map.panTo([lat, lon]);
      }, 100);
    }

    fetch('/mode')
      .then(res => res.json())
      .then(info => {
        if (info.mode === 'csv') {
          initCSVMode();
        } else {
          initLiveMode();
        }
      })
      .catch(() => initLiveMode());
  </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/data")
def data():
    return jsonify(latest_data)

@app.route("/data_csv")
def data_csv():
    return jsonify(data_trail)

@app.route("/mode")
def get_mode():
    return jsonify({"mode": mode})

def scan_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if "ACM" in p.device]

def read_serial(port):
    global latest_data, history_speeds
    ser = serial.Serial(port, 115200, timeout=1)
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(',')
            if len(parts) < 12:
                continue
            lat = float(parts[0])
            lon = float(parts[1])
            speed = float(parts[11])
            history_speeds.append(speed)
            vmax = max(history_speeds)
            vmean = sum(history_speeds) / len(history_speeds)
            formatted = f" lat: {lat}\n lon: {lon}\n speed: {speed:.2f} km/h\n Vmax: {vmax:.2f} km/h\n Vmean: {vmean:.2f} km/h"
            latest_data["lat"] = lat
            latest_data["lon"] = lon
            latest_data["speed"] = speed
            latest_data["formatted"] = formatted
        except Exception as e:
            print("Erreur:", e)

def read_csv(filepath):
    global data_trail
    with open(filepath, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for parts in reader:
            try:
                if len(parts) < 12:
                    continue
                lat = float(parts[0])
                lon = float(parts[1])
                speed = float(parts[11])
                formatted = f" lat: {lat}\n lon: {lon}\n speed: {speed:.2f} km/h"
                data_trail.append({
                    "lat": lat,
                    "lon": lon,
                    "speed": speed,
                    "formatted": formatted
                })
            except Exception:
                continue

def find_sd_mounts():
    candidates = []
    if platform.system() == "Linux":
        bases = ["/media", "/mnt", "/run/media"]
    elif platform.system() == "Darwin":
        bases = ["/Volumes"]
    elif platform.system() == "Windows":
        from string import ascii_uppercase
        bases = [f"{letter}:\" for letter in ascii_uppercase]
    else:
        return candidates
    for base in bases:
        if not os.path.exists(base):
            continue
        for root, dirs, files in os.walk(base):
            for file in files:
                if file.lower().endswith(".csv"):
                    candidates.append(os.path.join(root, file))
    return candidates

if __name__ == "__main__":
    mode = input("Choisir le mode [csv/live] : ").strip().lower()
    if mode == "csv":
        files = find_sd_mounts()
        if not files:
            print("Aucun fichier CSV trouvé.")
            exit(1)
        for i, f in enumerate(files):
            print(f"[{i}] {f}")
        idx = int(input("Choix du fichier : "))
        read_csv(files[idx])
        print(f"{len(data_trail)} points chargés.")
    elif mode == "live":
        ports = scan_ports()
        if not ports:
            print("Aucun port ACM trouvé.")
            exit(1)
        for i, p in enumerate(ports):
            print(f"[{i}] {p}")
        idx = int(input("Choisis un port : "))
        thread = threading.Thread(target=read_serial, args=(ports[idx],), daemon=True)
        thread.start()
    else:
        print("Mode invalide.")
        exit(1)

    app.run(debug=False, port=5000)
