import serial
import time
import sys

def test_uart_connection():
    port = '/dev/ttyUSB0'  # Anpassen wenn nötig
    baudrate = 115200
    
    print(f"Teste UART Verbindung auf {port} mit {baudrate} Baud")
    
    try:
        # Serielle Verbindung öffnen
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        if not ser.is_open:
            print("FEHLER: Konnte serielle Schnittstelle nicht öffnen!")
            return False
            
        print(f"✓ Serielle Schnittstelle {port} erfolgreich geöffnet")
        print("Warte auf Daten vom Pico... (Drücke Ctrl+C zum Beenden)")
        print("-" * 50)
        
        test_counter = 0
        
        while True:
            try:
                # Auf Daten vom Pico warten
                if ser.in_waiting > 0:
                    received = ser.readline().decode('utf-8').strip()
                    print(f"📥 VOM PICO: {received}")
                    
                    # Antwort an Pico senden
                    response = f"Laptop antwortet: Test #{test_counter}"
                    ser.write((response + '\n').encode())
                    print(f"📤 AN PICO: {response}")
                    
                    test_counter += 1
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\n\nTest beendet")
                break
            except Exception as e:
                print(f"Fehler während der Kommunikation: {e}")
                break
                
        ser.close()
        print("Serielle Schnittstelle geschlossen")
        return True
        
    except serial.SerialException as e:
        print(f"FEHLER: {e}")
        print("\nMögliche Lösungen:")
        print("1. Ist der Pico angeschlossen?")
        print("2. Ist der richtige Port angegeben?")
        print("3. Haben Sie Berechtigungen? (sudo chmod 666 /dev/ttyUSB0)")
        print("4. Ist ein anderes Programm auf dem Port?")
        return False
    except Exception as e:
        print(f"Unbekannter Fehler: {e}")
        return False

def list_serial_ports():
    """Listet verfügbare serielle Ports auf"""
    print("\nVerfügbare serielle Ports:")
    print("-" * 30)
    
    import glob
    ports = glob.glob('/dev/tty[A-Za-z]*')
    
    for port in ports:
        try:
            # Überprüfe ob es ein USB Gerät ist
            if 'USB' in port or 'ACM' in port:
                print(f"✓ {port} (Vermutlich Pico)")
            else:
                print(f"  {port}")
        except:
            print(f"  {port}")

if __name__ == "__main__":
    list_serial_ports()
    
    print("\n" + "=" * 50)
    
    # Teste Verbindung
    test_uart_connection()