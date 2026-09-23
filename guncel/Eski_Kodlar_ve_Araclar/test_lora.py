import serial
import time

# LINUX ICIN LORA PORTU (Genelde /dev/ttyUSB0 veya /dev/ttyUSB1 olur)
LORA_PORT = "/dev/ttyUSB0"  
BAUD_RATE = 9600

print("==================================================")
print(f"LoRa Test Programı Başlatılıyor... Port: {LORA_PORT}")
print("==================================================")
print("Bekleniyor... Dronelardan gelen veriler aşağıda akacaktır.")
print("(Çıkmak için Ctrl+C tuşlarına basabilirsiniz)\n")

try:
    with serial.Serial(LORA_PORT, BAUD_RATE, timeout=2) as ser:
        while True:
            line = ser.readline()
            if line:
                try:
                    data = line.decode('ascii', errors='ignore').strip()
                    if data:
                        # Linux terminali icin renklendirme kodlari
                        if data.startswith("N1"):
                            print(f"\033[92m[DRONE 1]\033[0m ---> {data}") # Yesil
                        elif data.startswith("N2"):
                            print(f"\033[94m[DRONE 2]\033[0m ---> {data}") # Mavi
                        elif data.startswith("N3"):
                            print(f"\033[93m[DRONE 3]\033[0m ---> {data}") # Sari
                        elif data.startswith("N4"):
                            print(f"\033[95m[DRONE 4]\033[0m ---> {data}") # Mor
                        else:
                            print(f"[DİĞER VERİ] -> {data}")
                except Exception as e:
                    print(f"[HATA] Bozuk paket: {e}")
            
except serial.SerialException:
    print(f"\n[BAĞLANTI HATASI] {LORA_PORT} portu açılamadı!")
    print("Lütfen Şunları Kontrol Edin:")
    print("1) Yeni bir terminal açıp 'ls /dev/ttyUSB*' yazarak portun gerçekten USB0 mı olduğunu doğrulayın.")
    print("2) İzin hatası (Permission Denied) alıyorsanız programı 'sudo python3 test_lora.py' olarak çalıştırın.")
except KeyboardInterrupt:
    print("\n\nTest programı durduruldu.")
