import serial.tools.list_ports
import folium
import webbrowser
import time
import os

def scan_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if "ACM" in p.device]

def choose_port():
    ports = scan_ports()
    if not ports:
        print("Aucun port ACM détecté.")
        return None
    print("Ports disponibles :")
    for i, port in enumerate(ports):
        print(f"[{i}] {port}")
    idx = int(input("Choisir le port à utiliser : "))
    return ports[idx]

def save_full_map(coordinates, path='carte_live.html'):
    if not coordinates:
        return
    m = folium.Map(location=coordinates[-1], zoom_start=15)
    for lat, lon in coordinates:
        folium.CircleMarker(location=[lat, lon], radius=2, color='red').add_to(m)
    if len(coordinates) > 1:
        folium.PolyLine(locations=coordinates, color='blue', weight=2.5).add_to(m)

    # Ajout d'un conteneur JS pour overlay dynamique
    info_box_html = """
    <div id='info-box' style='position: fixed; bottom: 20px; left: 20px; z-index: 9999;
                background-color: white; padding: 10px; border: 2px solid black;
                border-radius: 8px; font-size: 14px;'>
        Initialisation...
    </div>
    <script>
        function updateInfoBox(text) {
            document.getElementById('info-box').innerHTML = text;
        }
    </script>
    """
    m.get_root().html.add_child(folium.Element(info_box_html))
    m.save(path)

def inject_info_script(text, path='carte_live.html'):
    with open(path, 'r+') as f:
        content = f.read()
        if 'updateInfoBox(' in content:
            new_content = content.replace("Initialisation...", text)
            f.seek(0)
            f.write(new_content)
            f.truncate()

def main():
    port = choose_port()
    if not port:
        return

    ser = serial.Serial(port, 115200, timeout=1)
    print(f"Connecté à {port}")

    map_path = "carte_live.html"
    browser_opened = False
    coordinates = []

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(',')
            try:
                lat = float(parts[0])
                lon = float(parts[1])
                pdop = parts[10]
                gyro = f"gyro: {parts[14]} / {parts[15]} / {parts[16]}"
                temp = parts[31]
                time_str = f"{parts[6]}h{parts[7]}m{parts[8]}s{parts[9]}ms"
                info_html = f"{time_str}<br>PDOP: {pdop}<br>{gyro}<br>Temp: {temp}°C"

                coordinates.append([lat, lon])

                if not browser_opened:
                    save_full_map(coordinates, map_path)
                    inject_info_script(info_html, map_path)
                    webbrowser.open('file://' + os.path.realpath(map_path))
                    browser_opened = True
                else:
                    inject_info_script(info_html, map_path)

                print(f"Point ajouté : {lat}, {lon}")

            except Exception as e:
                print(f"Erreur parsing ligne : {line}\n{e}")
    except KeyboardInterrupt:
        print("Arrêté par l'utilisateur.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
