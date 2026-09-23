# TÜBİTAK 123E294 - PROJE BİLGİ AKTARIMI (AI CONTEXT)

Bu dosya, bu projeyi devralacak veya analiz edecek bir yapay zeka (LLM) modeli için projenin başından sonuna kadar olan tüm bağlamı, donanımı ve algoritmaları özetlemek amacıyla oluşturulmuştur.

## 1. PROJE ÖZETİ VE DONANIM
- **Proje:** TÜBİTAK 123E294 - RF Sinyal Gücü (RSSI) ile Otonom İHA Sürüsü Yönlendirme ve Hedef Tespiti.
- **Hedef Sinyal:** 446.148 MHz (PMR El Telsizi).
- **İHA Donanımı:** 4 Adet Quadcopter (Pixhawk/ArduPilot otopilot), Görev Bilgisayarı (Raspberry Pi vb.), RTL-SDR (Sinyal alıcı), CH340 çipli LoRa (Yer istasyonuyla haberleşme).
- **Yazılım Mimarisi:** Python 3.12, pymavlink, pyrtlsdr. İHA'lar "GUIDED" modunda uçurulmakta ve "SET_POSITION_TARGET_GLOBAL_INT" komutlarıyla yönlendirilmektedir.

## 2. ÇÖZÜLMÜŞ KRİTİK DONANIMSAL SORUNLAR (BUNLARI TEKRARLAMA)
1. **Anten Doygunluğu (Saturation):** Hedef İHA'lara çok yaklaştığında SDR alıcıları doyuma ulaşıyor ve sinyal gradyanı kayboluyordu. **Çözüm:** SDR antenleri fiziksel olarak söküldü. Sadece cihazın kendi PCB devre hatları (ve İHA gövdesinin sinyali zayıflatma etkisi) kullanılarak yakın alan ölçüm aralığı (dynamic range) optimize edildi.
2. **SDR Linux Çakışması:** Linux çekirdeği SDR'ı DVB-T (Karasal TV) alıcısı sanıp kilitliyordu. **Çözüm:** `/etc/modprobe.d/blacklist-rtl.conf` dosyasına `blacklist rtl2832` ve `blacklist dvb_usb_rtl28xxu` eklenerek çözüldü.
3. **USB Güç Çökmesi (Brownout) ve EMI:** LoRa modülü TX (veri gönderme) anında yüksek akım çektiği için, USB çoklayıcı (Hub) kullanıldığında SDR voltajı düşüyor ve sistem `failed with -4 (LIBUSB_ERROR_NO_DEVICE)` ile çöküyordu. **Çözüm:** USB çoklayıcı iptal edildi, SDR ve LoRa doğrudan görev bilgisayarının ayrı portlarına bağlandı ve harici güçlü BEC ile beslendi. EMI parazitini önlemek için LoRa anteni işlemciden uzaklaştırıldı.

## 3. KALİBRASYON MODELİ
- **Algoritma:** 13 Noktalı (Merkez + 4 yön x 3 farklı mesafe) konumsal ortalamalı kalibrasyon uygulandı (Numpy Least Squares).
- **Güncel Değerler:** RMSE = 2.29 dB, Path Loss (n) = 1.264, P_ref = -78.72 dBm.
- **Donanım Ofsetleri:** D1: 0.0, D2: -1.32, D3: +2.38, D4: +1.10 dBm. (Uçuş kararlarında bu ofsetler uygulanarak 4 İHA'nın sağırlık farkları eşitlenir).

## 4. OTONOM SÜRÜ KARAR MOTORU VE ALGORİTMALAR
### A. Ham Veri Filtreleme
Gövde yansıması ve dönüşlerdeki sinyal dalgalanmalarını (multipath) engellemek için, gelen her ham RSSI değeri Skaler **Kalman Filtresinden** (Q=1.0, R=20.0) geçirilir.

### B. Küre (Sphere) Formasyonu (3D Lokalizasyon)
- **Mantık:** 4 drone iki gruba ayrılır. Grup 1 (D1, D3) yatay düzlemde (X-Y) çember çizerken, Grup 2 (D2, D4) dikey düzlemde (X-Z) irtifa değiştirerek çember çizer.
- **PID ve Yönelim (Orbit):** Dronlar hedefin nerede olduğunu doğrudan GPS ile hesaplamaz. Güç farkları (P3-P1 ve P4-P2) bir PID kontrolcüye verilir. PID, dronları sinyal farkı sıfırlanana kadar (hedefe hizalanana kadar) küre üzerinde döndürür ($\alpha$ ve $\beta$ açıları).
- **İlerleme (Converged):** Güç farkları sıfırlandığında, hedefin 3 boyutlu yönünü gösteren bir "Artırılmış Nokta (Augmented Point)" hesaplanır. Formasyonun merkezi bu yöne doğru kaydırılır.

### C. Arayüz (GUI)
Tkinter ile yazılmış `Kalibreli_Radar_GUI.py` mevcuttur. Log-Normal formülü ($d = 10^{(P_{ref} - RSSI) / 10n}$) kullanılarak hedefin metrik mesafesi hesaplanır ve ekranda dronların çok dışında dahi olsa doğru konumda (trail iziyle birlikte) çizilir.

## SON DURUM
Yazılım testleri başarıyla geçmiştir. Şu an sistem donanımsal olarak otonom uçuşlara hazır durumdadır. Tüm uçuş kodları `Guncel_Proje_Dosyalari/Yer_Istasyonu_Kodlari` altındadır.
