import serial
import time

# BURAYI KENDİ BİLGİSAYARINIZDAKİ YER LORA'SININ COM PORTU İLE DEĞİŞTİRİN
LORA_PORT = "/dev/ttyUSB0" # Örnek: "COM5", "COM8" vs. (Aygıt yöneticisinden bakın)
BAUD_RATE = 9600

print("==================================================")
print(f"LoRa Test Programı Başlatılıyor... Port: {LORA_PORT}")
print("==================================================")
print("Bekleniyor... Dronelardan gelen veriler aşağıda akacaktır.")
print("(Çıkmak için Ctrl+C tuşlarına basabilirsiniz)\n")

try:
    # Seri portu aç (timeout=2 saniye bekleme süresi)
    with serial.Serial(LORA_PORT, BAUD_RATE, timeout=2) as ser:
        while True:
            # Seri porttan gelen satırı oku
            line = ser.readline()
            
            # Eğer veri geldiyse ekrana yazdır
            if line:
                try:
                    # Gelen ham veriyi okunabilir metne (string) çevir
                    data = line.decode('ascii', errors='ignore').strip()
                    
                    if data:
                        # Gelen paketin başına görsellik ekle
                        if data.startswith("N1"):
                            print(f"[DRONE 1] ---> {data}")
                        elif data.startswith("N2"):
                            print(f"[DRONE 2] ---> {data}")
                        elif data.startswith("N3"):
                            print(f"[DRONE 3] ---> {data}")
                        elif data.startswith("N4"):
                            print(f"[DRONE 4] ---> {data}")
                        else:
                            print(f"[DİĞER VERİ] -> {data}")
                            
                except Exception as e:
                    print(f"[HATA] Bozuk paket: {e}")
            
except serial.SerialException:
    print(f"\n[BAĞLANTI HATASI] {LORA_PORT} portu açılamadı!")
    print("Lütfen Şunları Kontrol Edin:")
    print("1) Lütfen kodun içindeki LORA_PORT kısmına doğru COM portunu yazdığınızdan emin olun.")
    print("2) Arka planda RealTerm veya Mission Planner gibi bu portu işgal eden bir program açık OLMAMALI!")
except KeyboardInterrupt:
    print("\n\nTest programı sizin tarafınızdan durduruldu. İyi çalışmalar!")