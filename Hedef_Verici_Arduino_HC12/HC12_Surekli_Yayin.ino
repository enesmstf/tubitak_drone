#include <SoftwareSerial.h>

// RX ve TX pinlerini Arduino'nuza gore degistirebilirsiniz.
// Su an HC-12'nin TX pini Arduino'nun 2'sine, HC-12'nin RX pini Arduino'nun 3'une bagli varsayilmistir.
SoftwareSerial HC12(2, 3); 

void setup() {
  Serial.begin(9600);
  HC12.begin(9600);
  
  Serial.println("HC-12 Sinyal Feneri (Beacon) Baslatildi.");
  Serial.println("Havaya kesintisiz enerji basiliyor...");
}

void loop() {
  // Havada sürekli ve kesintisiz bir radyo frekans enerjisi (peak) 
  // oluşturmak için durmaksızın tek baytlık bir karakter basıyoruz.
  HC12.write(0xAA); 
  
  // Telsiz mandali gibi sürekli basili tutma etkisi yaratir
  // 5 milisaniyelik gecikme HC-12'nin buffer'ini bogmamak icindir.
  delay(5); 
}
