import serial
import folium

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyACM0'  # ou COM3 sous Windows
BAUD_RATE = 115200
MAP_OUTPUT = 'carte_live.html'

# Initialisation de la carte
start_coords = [48.0, -1.0]  # Valeurs initiales arbitraires
m = folium.Map(location=start_coords, zoom_start=13)
points = []

# Connexion série
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
print(f"Connexion série ouverte sur {SERIAL_PORT} à {BAUD_RATE} bauds.")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line or 'lat' not in line:
            continue
        try:
            parts = line.split(',')
            lat = float(parts[0])
            lon = float(parts[1])
            folium.CircleMarker(location=[lat, lon], radius=2, color='red').add_to(m)
            points.append((lat, lon))
            print(f"Point ajouté : {lat}, {lon}")
        except Exception as e:
            print(f"Erreur de parsing : {e}")
except KeyboardInterrupt:
    print("Interruption par utilisateur.")
finally:
    ser.close()
    m.save(MAP_OUTPUT)
    print(f"Carte sauvegardée dans : {MAP_OUTPUT}")
