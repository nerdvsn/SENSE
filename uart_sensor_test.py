import serial
import json
import numpy as np

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

print("Warte auf MLX90640 Daten...")
print("Drücke Ctrl+C zum Beenden")

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            
            try:
                # Versuche JSON zu parsen
                data = json.loads(line)
                
                if "temperature" in data:
                    temp_array = np.array(data["temperature"])
                    ambient = data["at"]
                    
                    print(f"✅ Empfangen: {len(temp_array)} Werte")
                    print(f"   Ambient: {ambient}°C")
                    print(f"   Min: {min(temp_array):.1f}°C, Max: {max(temp_array):.1f}°C")
                    
                    # Für dein Hauptprogramm:
                    # temperature_matrix = temp_array.reshape((24, 32))
                    
            except json.JSONDecodeError:
                print(f"Raw: {line[:50]}...")
                
except KeyboardInterrupt:
    print("\nBeendet")
    ser.close()