import serial
import time

LORA_PORT = "/dev/ttyUSB0"  
BAUD_RATE = 9600

print("==================================================")
print("Karar Motoru LoRa Testi (v3 - Şifreli Çözücü)")
print("==================================================")

try:
    with serial.Serial(LORA_PORT, BAUD_RATE, timeout=2) as ser:
        while True:
            line = ser.readline()
            if line:
                try:
                    data = line.decode('ascii', errors='ignore').strip()
                    if not data:
                        continue
                        
                    # Yeni kodun attığı şifreli veriyi parçala (Örn: N2,-75,123)
                    parts = data.split(',')
                    if len(parts) == 3 and parts[0].startswith("N"):
                        drone_id = parts[0][1:] # 'N2' -> '2'
                        rssi_val = int(parts[1])
                        received_checksum = int(parts[2])
                        
                        # Karar Motorunun yaptığı şifre kırma (Checksum) işlemi
                        base_msg = f"{parts[0]},{parts[1]}"
                        calculated_checksum = sum(base_msg.encode('ascii')) % 256
                        
                        if received_checksum == calculated_checksum:
                            print(f"[\033[92mŞİFRE ONAYLANDI\033[0m] Drone {drone_id} -> Sinyal Gücü: {rssi_val} dBm")
                        else:
                            print(f"[\033[91mŞİFRE HATALI\033[0m] Drone {drone_id} paketi havada bozulmuş! (Çöpe atıldı)")
                    else:
                        print(f"[ESKİ/BOZUK VERİ] -> {data}")
                        
                except Exception as e:
                    pass
            
except serial.SerialException:
    print(f"\n[BAĞLANTI HATASI] {LORA_PORT} portu açılamadı!")
except KeyboardInterrupt:
    print("\nTest durduruldu.")
