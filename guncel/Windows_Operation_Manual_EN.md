# Autonomous Drone Swarm RF Tracking System
## Operation & Testing Manual (Windows Edition)

This manual provides step-by-step instructions for deploying the RF Tracking System on 4 autonomous drones and a Windows Ground Station.

> [!IMPORTANT]
> Do NOT open RealTerm or any other serial monitor while running the Ground Station Python script. Windows only allows one program to access a COM port at a time. The Python script includes a built-in monitor that replaces RealTerm.

---

### 1. Drone Preparation (Raspberry Pi Setup)
You must upload the `Check1_v3.py` (or `Check1_yeni.py`) file to the Raspberry Pi Zero on each of the 4 drones.

**Step 1.1: Transfer the Code**
Copy the Python script to the Raspberry Pi of each drone.

**Step 1.2: Assign Unique Node IDs**
Before running the script, you **MUST** open the file in a text editor (e.g., `nano Check1_v3.py`) and change the `NODE_ID` on Line 19 for each specific drone:
- Drone 1 (North): `NODE_ID = 1`
- Drone 2 (South): `NODE_ID = 2`
- Drone 3 (East): `NODE_ID = 3`
- Drone 4 (West): `NODE_ID = 4`

> [!CAUTION]
> If two drones have the same `NODE_ID`, their radio transmissions will collide, and the Ground Station will reject their data. 

**Step 1.3: Start the Drones**
Run the script on each drone:
```bash
python3 Check1_v3.py
```
The drones will automatically connect to their SDR and LoRa modules and begin transmitting their measured RSSI values asynchronously.

---

### 2. Ground Station Setup (Windows)

**Step 2.1: Hardware Connections**
Plug the following USB modules into your Windows computer:
- **1x LoRa Module** (To receive RSSI data from the drones)
- **4x Pixhawk Telemetry Radios** (To send MAVLink commands to each drone)

**Step 2.2: Identify COM Ports**
1. Right-click the Windows Start Button and open **Device Manager**.
2. Expand the **Ports (COM & LPT)** section.
3. Note the COM port numbers (e.g., `COM5`, `COM6`, etc.) for your connected USB devices.

**Step 2.3: Configure the Script**
Open `ground_station_windows.py` in a text editor (like VS Code or Notepad). Edit the port settings at the top of the file to match your Device Manager findings:
```python
LORA_PORT = "COM5"

PIXHAWK_PORTS = {
    1: "COM6",  # Drone 1
    2: "COM7",  # Drone 2
    3: "COM8",  # Drone 3
    4: "COM9",  # Drone 4
}
```

**Step 2.4: Execute the System**
Open Command Prompt (CMD) or PowerShell, navigate to your project folder, and run:
```cmd
python ground_station_windows.py
```

---

### 3. What You Will See on the Screen (The Interface)

The Python console will act as your new RealTerm, showing synchronized data flows and mathematical decisions in real-time.

**Phase 1: Initialization**
```text
[SİSTEM] Gerçek LoRa portuna bağlanılıyor: COM5...
[BAŞARILI] LoRa Bağlandı: COM5
[BAĞLANTI] Drone 1 portu açıldı: COM6. Heartbeat bekleniyor...
[BAŞARILI] Drone 1 Heartbeat alındı (sysid=1, compid=1)
[SİSTEM] Origin için Drone 1'in GPS konumu bekleniyor...
[BAŞARILI] Origin (Drone1'den 5.0m güneye kaydırıldı) -> LAT: 40.123456, LON: 30.123456
```

**Phase 2: Synchronized RSSI Stream (RealTerm Alternative)**
You will clearly see the dBm values of the central HC-12 beacon arriving from each drone:
```text
[RADYO-MONITOR] N1,-75,76
[RADYO-MONITOR] N3,-68,71
[RADYO-MONITOR] N2,-78,80
[RADYO-MONITOR] N4,-70,74
```

**Phase 3: Decision Engine & MAVLink (1 Hz Loop)**
Every second, the system will process the raw data and output the calculated flight paths:
```text
=======================================================
[KARAR MOTORU] Sinyal Farkları -> K/G: 3dB, D/B: 2dB
[KARAR MOTORU] Adım Kayması -> KUZEYE: 1.5m, DOĞUYA: 1.0m
[KARAR MOTORU] Sanal Merkez (Origin'e göre) -> KUZEY: 1.5m, DOĞU: 1.0m (Geofence: ±20.0m)
[MAVLINK] Drone 1 -> LAT: 40.123512, LON: 30.123467, ALT: 15.0m
[MAVLINK] Drone 2 -> LAT: 40.123400, LON: 30.123467, ALT: 15.0m
...
=======================================================
```

---

### 4. Pre-Flight Checklist & Testing

Before enabling autonomous flight, test the following aspects on the ground:

- [ ] **RF Signal Validation:** Move the central HC-12 beacon closer to Drone 1 (North). Watch the Windows screen. The RSSI for `N1` should become stronger (e.g., jump from `-85` to `-60`). The Decision Engine should calculate a positive North shift.
- [ ] **GPS Validation:** Ensure Drone 1 is placed outside with a clear sky view. The system will NOT start the main loop until Drone 1 acquires a 3D GPS lock (`[BAŞARILI] Origin ayarlandı`).
- [ ] **Geofence Safety Check:** Take the central HC-12 beacon and walk 50 meters away. Check the screen; the Virtual Center (`Sanal Merkez`) should hit the Geofence limit (`KUZEY: 20.0m`) and refuse to increase further.

---

### 5. Troubleshooting Solution Map

| Symptom / Error Message | Root Cause | Solution |
| :--- | :--- | :--- |
| **"Access Denied"** on LoRa port | The COM port is being used by another app. | Close RealTerm, Mission Planner, or any other software that might be connected to that specific COM port. |
| **"Heartbeat kaybı"** or **Heartbeat timeout** | The Pixhawk Telemetry is not communicating with the PC. | 1. Check if the drone is powered. 2. Verify Pixhawk `SERIALx_BAUD` matches `57600`. 3. Check USB cables. |
| **"Origin henüz ayarlanmadı"** loops endlessly | Drone 1 (North) cannot find GPS satellites. | Move Drone 1 to an open field. Wait for the Pixhawk LED to turn green (3D Fix). |
| **"Checksum uyuşmadı, paket reddedildi"** | RF Interference or incorrect Baud Rate. | This means corrupt data was blocked safely. If it happens constantly, keep your LoRa antennas away from dense electronics/metal. |
| **"Drone X komuta hazır değil (havada değil)"** | Safety lock preventing drones from going rogue on the ground. | The code requires drones to be >2m in the air. Manually take off, reach 3 meters, switch to **GUIDED** mode, and the code will take over. |
| **Compass Variance / EKF errors** on Pixhawk | The 433 MHz SDR or LoRa is magnetically jamming the Pixhawk. | Move the HC-12, SDR, and LoRa modules (and their antennas) to the extremities of the drone arms, far from the Pixhawk cube. |

> [!TIP]
> Always configure ArduPilot's **GCS Failsafe** (via Mission Planner). Set it to **RTL** or **LAND** if Ground Station telemetry is lost for 5 seconds. This ensures drones return safely if your Windows laptop crashes.
