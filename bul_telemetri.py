import sys
import glob
from pymavlink import mavutil

def find_telemetry_ports():
    print("==================================================")
    print("TELEMETRİ (PİXHAWK) OTOMATİK TARAMA BAŞLIYOR...")
    print("==================================================")
    
    # Tüm USB portlarını bul, ttyUSB0 (LoRa) hariç
    ports = glob.glob('/dev/ttyUSB*')
    if '/dev/ttyUSB0' in ports:
        ports.remove('/dev/ttyUSB0')
        
    print(f"Taranacak Telemetri Portları: {ports}")
    
    mapping = {}
    for port in ports:
        print(f"\n{port} açılıyor, Heartbeat (kalp atışı) bekleniyor...")
        try:
            # 57600 baud rate ile Pixhawk'a bağlan
            connection = mavutil.mavlink_connection(port, baud=57600)
            msg = connection.wait_heartbeat(timeout=10)
            
            if msg:
                sysid = connection.target_system
                print(f"[\033[92mBAŞARILI\033[0m] {port} portunun ucunda DRONE {sysid} var!")
                mapping[sysid] = port
            else:
                print(f"[\033[91mBAŞARISIZ\033[0m] {port} portundan ses gelmiyor (Cihaz kapalı olabilir).")
        except Exception as e:
            print(f"[\033[91mHATA\033[0m] {port} portuna bağlanılamadı: {e}")
            
    print("\n==================================================")
    print("KODA YAPIŞTIRMANIZ GEREKEN DOĞRU SÖZLÜK AŞAĞIDADIR:")
    print("==================================================")
    print("PIXHAWK_PORTS = {")
    for drone_id in sorted(mapping.keys()):
        print(f"    {drone_id}: \"{mapping[drone_id]}\",")
    print("}")

if __name__ == "__main__":
    find_telemetry_ports()
