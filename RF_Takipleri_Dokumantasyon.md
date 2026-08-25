# TÜBİTAK Sürü Drone - RF Sinyal Takip Kodları

Bu klasörde, dronların yerdeki bir radyo frekans hedefini (Beacon) RTL-SDR kullanarak takip edebilmesi için hazırlanmış iki farklı alternatif kod bulunmaktadır.

## 1. El Telsizi (PMR446) İle Takip
**Dosya:** `Check1_PMR_Telsiz.py`

*   **Frekans:** 446.148 MHz (PMR Kanal 12 vb.)
*   **Donanım Özelliği:** El telsizleri çok yüksek güçlüdür (yaklaşık 1 Watt).
*   **Kod Farklılıkları:**
    *   `SDR_TARGET_GAIN_DB = 10.0` olarak ayarlanmıştır. Telsiz çok güçlü olduğu için SDR çipinin sağırlaşmasını (saturation) önlemek amacıyla kazanç (gain) çok düşük tutulmuştur.
    *   Filtre penceresi dar bant FM (NFM) dalgalanmalarını kapsayacak şekilde `+- 15 kHz` olarak ayarlanmıştır.
*   **Test Önerisi:** Telsiz çok güçlü olduğu için iç mekan testlerinde dronların üzerindeki RTL-SDR antenlerini çıkarmanız veya kazancı `0.0` yapmanız önerilir.

## 2. HC-12 Modülü İle Takip
**Dosya:** `Check1_HC12.py` ve `Hedef_Verici_Arduino_HC12/HC12_Surekli_Yayin.ino`

*   **Frekans:** 433.4 MHz (HC-12 Varsayılan frekansı)
*   **Donanım Özelliği:** HC-12 modülü düşük güçlüdür (maksimum 100 mW).
*   **Kod Farklılıkları:**
    *   `SDR_TARGET_GAIN_DB = 25.0` olarak ayarlanmıştır. Sinyal zayıf olduğu için SDR kazancı yüksek tutulmalıdır.
    *   Filtre penceresi GFSK modülasyonu için `+- 20 kHz` olarak ayarlanmıştır.
*   **Kullanım Şartı (Sinyal Feneri):** HC-12'nin telsiz gibi kesintisiz yayın yapabilmesi için, hedef olarak kullanılacak HC-12'nin bağlı olduğu Arduino'ya kesinlikle `HC12_Surekli_Yayin.ino` kodu yüklenmelidir. Aksi takdirde modül anlık (milisaniyelik) yayın yapar ve SDR bu yayını kaçırarak sürekli `0` değeri üretir.

## Ortak Kurallar
Her iki kod da sürü hiyerarşisi (TDMA) ve LoRa telemetrisi ile tam uyumludur. Hangi dosyayı kullanacaksanız o dosyanın içindeki `NODE_ID = 1` değerini, yükleme yapacağınız drone'un numarasına göre (1, 2, 3, 4) değiştirmeyi ve dosyanın adını drone içinde `Check1.py` olarak kaydetmeyi unutmayın.

Ayrıca RSSI testlerinde en doğru sonucu almak için tüm dronların donanım konfigürasyonlarının (anten varlığı/yokluğu, anten açıları) birebir aynı olmasına dikkat edilmelidir.
