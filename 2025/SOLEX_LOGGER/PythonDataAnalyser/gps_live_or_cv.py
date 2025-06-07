from flask import Flask, render_template, request, redirect, jsonify
import serial.tools.list_ports
import serial
import threading
import csv
import os

app = Flask(__name__)
latest_data = {"lat": None, "lon": None, "speed": 0, "formatted": "En attente..."}
all_points = []
use_csv_mode = False # true = csv, false = serie

headers = [
    "lat", "long", "height", "day", "month", "year", "hour", "minute", "second", "millis", "pdop", "speed_kmh",
    "ori_x", "ori_y", "ori_z", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
    "mag_x", "mag_y", "mag_z", "lin_x", "lin_y", "lin_z", "grav_x", "grav_y", "grav_z",
    "temp", "cal_sys", "cal_gyro", "cal_accel", "cal_mag"
]

@app.route("/")
def index():
    ports = [p.device for p in serial.tools.list_ports.comports() if "ACM" in p.device]
    return render_template("index.html", ports=ports)

@app.route("/live", methods=["POST"])
def start_live():
    global use_csv_mode
    use_csv_mode = False
    port = request.form["port"]
    thread = threading.Thread(target=read_serial, args=(port,), daemon=True)
    thread.start()
    return redirect("/map")

@app.route("/upload", methods=["POST"])
def upload_file():
    global use_csv_mode
    use_csv_mode = True
    file = request.files["csvfile"]
    path = "/tmp/uploaded.csv"
    file.save(path)
    load_csv(path)
    return redirect("/map")

@app.route("/map")
def map_view():
    return render_template("map.html")

@app.route("/data")
def data():
    return jsonify(all_points if use_csv_mode else [latest_data])

def read_serial(port):
    global latest_data
    ser = serial.Serial(port, 115200, timeout=1)
    history = []
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(',')
            lat = float(parts[0])
            lon = float(parts[1])
            speed = float(parts[11])
            millis = int(parts[9])
            history.append(speed)
            vmax = max(history)
            vmean = sum(history) / len(history)
            formatted = ""
            for h, v in zip(headers, parts):
                formatted += f"{h:>10}: {v}\n"
            formatted += f"\n    Vitesse max: {vmax:.2f} km/h"
            formatted += f"\n Vitesse moyenne: {vmean:.2f} km/h"
            latest_data.update({
                "lat": lat,
                "lon": lon,
                "speed": speed,
                "formatted": formatted
            })
        except Exception as e:
            print("Erreur:", e)

def load_csv(path):
    global all_points
    all_points = []
    history = []
    with open(path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader, None)
        for parts in reader:
            try:
                lat = float(parts[0])
                lon = float(parts[1])
                millis = int(parts[9])
                speed = float(parts[11])
                history.append(speed)
                vmax = max(history)
                vmean = sum(history) / len(history)
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
                    "formatted": formatted
                })
            except:
                continue

if __name__ == "__main__":
    app.run(debug=True, port=5000)
