
## **1 Kamera einrichten**    
- Wenn die Kamera **weit weg vom PC** ist → **IP-Kamera** (unser Fall)

### **So richtest du die Kamera ein:**  
**IP-Kamera:**  
- Kamera mit Strom & Netzwerk verbinden (PoE oder WLAN)  
- Im Webbrowser die IP-Adresse der Kamera eingeben  
- Einstellungen vornehmen (Auflösung, Stream aktivieren)  

---

## **2 Computer vorbereiten **  
Du brauchst einen PC oder Mini-PC (z. B. Intel NUC, Raspberry Pi geht auch für einfachere Anwendungen).  

**Software installieren (auf Linux oder Windows):**  
1. Installiere Python (falls noch nicht vorhanden)  
2. Installiere die benötigten Pakete:  
   ```bash
   pip install opencv-python ultralytics
   ```
3. Teste, ob die Kamera erkannt wird:  
   ```python
   import cv2
   cap = cv2.VideoCapture(0)  # Für USB-Kamera
   while True:
       ret, frame = cap.read()
       if not ret:
           break
       cv2.imshow("Kamera", frame)
       if cv2.waitKey(1) & 0xFF == ord('q'):
           break
   cap.release()
   cv2.destroyAllWindows()
   ```
   - Falls du eine **IP-Kamera** nutzt, ersetze `0` durch die RTSP-URL der Kamera.  
     Beispiel:  
     ```python
     cap = cv2.VideoCapture("rtsp://benutzer:passwort@ip-adresse:554/stream")
     ```

---

## **KI-Modell für Objekterkennung **  
Wir nutzen **YOLOv8**, ein fertiges Modell zur Objekterkennung.  

### **YOLO-Modell herunterladen:**  
```bash
pip install ultralytics
```

### **Objekterkennung starten:**  
```python
from ultralytics import YOLO
import cv2

# Modell laden
model = YOLO("yolov8n.pt")  # Fertiges Modell für Objekterkennung

# Kamera-Stream starten
cap = cv2.VideoCapture(0)  # USB-Kamera (für IP-Kamera RTSP-Link einfügen)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO-Detektion
    results = model(frame)

    # Ergebnisse zeichnen
    for result in results:
        frame = result.plot()

    cv2.imshow("Erkennung", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
```

Wenn du das startest, erkennt das Programm Autos 🚗, Fahrräder 🚲 und Fußgänger 🚶‍♂️ in Echtzeit! 🎉  

---

## **Erkennung speichern (14 Tage) 💾**  
### **Speicherung der Videos:**  
```python
import datetime
import os

# Speicherordner
save_path = "videos"
os.makedirs(save_path, exist_ok=True)

# Video-Writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
out = cv2.VideoWriter(f"{save_path}/{now}.avi", fourcc, 20.0, (640, 480))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    frame = results[0].plot()

    out.write(frame)  # Video speichern

    cv2.imshow("Erkennung", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
```

### **Alte Videos nach 14 Tagen löschen:**  
Erstelle ein Skript `cleanup.py`:  
```python
import os
import time

folder = "videos"
days = 14
now = time.time()

for file in os.listdir(folder):
    filepath = os.path.join(folder, file)
    if os.stat(filepath).st_mtime < now - days * 86400:
        os.remove(filepath)
        print(f"Gelöscht: {filepath}")
```
Lass das Skript per **Cronjob (Linux)** oder **Geplante Aufgaben (Windows)** jeden Tag laufen.  

---

## **Ampelsteuerung**  
Falls du eine Ampel steuern willst, kannst du einen **ESP32 oder Raspberry Pi** verwenden. Der PC schickt ein Signal, wenn ein Auto erkannt wird.  

Beispiel:  
```python
import paho.mqtt.client as mqtt

mqtt_broker = "192.168.1.100"  # IP des ESP32/Raspberry Pi
client = mqtt.Client()
client.connect(mqtt_broker)

while True:
    ret, frame = cap.read()
    results = model(frame)

    # Falls ein Auto erkannt wird → Ampel schalten
    for result in results:
        if "car" in result.names.values():
            client.publish("ampel", "gruen")
        else:
            client.publish("ampel", "rot")
```

Auf dem **ESP32 oder Raspberry Pi** wird dann ein MQTT-Listener laufen, der die Ampel steuert.  

---

