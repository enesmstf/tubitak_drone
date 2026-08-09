"""
spectrum_debug.py

Tek seferlik tanı aracı. Bir drone'da ÇALIŞTIRIN (Check1.py'yi durdurun,
aynı anda ikisi RTL-SDR'ı açamaz).

İKİ KEZ çalıştırın:
  1) Merkez verici KAPALI iken:  python3 spectrum_debug.py > spectrum_off.txt
  2) Merkez verici drone'a yakın ve AÇIK iken: python3 spectrum_debug.py > spectrum_on.txt

Sonra iki .txt dosyasının içeriğini paylaşın. En güçlü 15 frekans bin'ini,
DC'den (SDR_FREQ merkezinden) ne kadar uzakta olduklarını ve güçlerini
gösterir. Bu, gerçek hedef sinyalin nerede olduğunu ve varsa sabit
parazitlerin (spur) nerede olduğunu netleştirir.
"""

import numpy as np
from rtlsdr import RtlSdr

SDR_FREQ = 433.0e6
SDR_SAMPLE_RATE = 2.048e6
N = 16384
TARGET_GAIN_DB = 40.0
NUM_FRAMES = 8  # birden fazla frame ortalayarak gürültüyü biraz düzeltiyoruz

print(f"# SDR_FREQ = {SDR_FREQ/1e6:.4f} MHz, SAMPLE_RATE = {SDR_SAMPLE_RATE/1e6:.4f} MHz, N = {N}")

sdr = RtlSdr()
sdr.sample_rate = SDR_SAMPLE_RATE
sdr.center_freq = SDR_FREQ

# PLL bazi RTL-SDR klonlarinda ilk denemede kilitlenmeyebilir.
# Ayni frekansi kisa bir bekleme sonrasi tekrar set etmek genelde
# PLL'i yeniden kalibre edip kilitlenmesini saglar.
import time as _time
_time.sleep(0.3)
sdr.center_freq = SDR_FREQ
_time.sleep(0.2)

try:
    sdr.set_manual_gain_enabled(True)
except Exception as e:
    print(f"# UYARI: set_manual_gain_enabled basarisiz: {e}")

valid_gains = sdr.valid_gains_db
closest_gain = min(valid_gains, key=lambda g: abs(g - TARGET_GAIN_DB))
sdr.gain = closest_gain
print(f"# Gain sabitlendi: {closest_gain} dB")

# DC/USB transient'i atla
_ = sdr.read_samples(4096)

# Birden fazla frame'in gücünü ortalayarak (Welch benzeri) daha kararlı bir spektrum al
accum_power = np.zeros(N)
window = np.hanning(N)

for _ in range(NUM_FRAMES):
    samples = sdr.read_samples(N)
    windowed = samples * window
    fft_data = np.fft.fftshift(np.fft.fft(windowed))
    power = np.abs(fft_data) ** 2
    accum_power += power

avg_power = accum_power / NUM_FRAMES

# fftshift sonrası bin 0 en düşük frekansı, bin N-1 en yüksek frekansı temsil eder
# DC (merkez frekans) tam ortadaki bin'e denk gelir
freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / SDR_SAMPLE_RATE))

# DC etrafındaki birkaç bin'i (gerçek DC spike/offset) ayrıca işaretleyelim
dc_bin = N // 2

# En güçlü 15 bin'i bul
top_indices = np.argsort(avg_power)[::-1][:15]

print(f"\n# En güçlü 15 bin (güç dB, DC'den frekans farkı Hz, bin index, DC'ye mesafe bin sayısı olarak):")
print(f"{'Sıra':>4} | {'dB':>8} | {'DC_offset_Hz':>14} | {'bin_idx':>8} | {'bin_DC_fark':>12}")
for rank, idx in enumerate(top_indices, 1):
    dbval = 10 * np.log10(avg_power[idx] + 1e-12)
    offset_hz = freqs[idx]
    print(f"{rank:>4} | {dbval:>8.2f} | {offset_hz:>14.1f} | {idx:>8} | {idx - dc_bin:>12}")

# DC bin'in kendisi ve hemen komşularının gücü (referans için)
print(f"\n# DC (merkez frekans) bin'i ve komsulari:")
for offset in range(-3, 4):
    idx = dc_bin + offset
    dbval = 10 * np.log10(avg_power[idx] + 1e-12)
    print(f"  DC{offset:+d} (bin {idx}): {dbval:.2f} dB")

sdr.close()
print("\n# Tamamlandi.")
