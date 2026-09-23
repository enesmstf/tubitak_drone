import serial
import time

PORT = "COM5"
BAUD = 9600

TARGET_ID = 2

REQ_REPEAT = 6
REQ_INTERVAL = 0.20
RESPONSE_TIMEOUT = 3.0

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.05,
    write_timeout=1
)

# DTR/RTS'e şimdilik dokunmuyoruz.
# Bazı adaptörlerde M0/M1 pinlerine etkisi olabiliyor.

rx_buffer = ""

print("Ground polling test started on", PORT)
print(f"Target drone: N{TARGET_ID}")

def read_lines_until(deadline):
    global rx_buffer

    lines = []

    while time.monotonic() < deadline:
        data = ser.read(256)

        if data:
            text = data.decode(errors="ignore")
            rx_buffer += text

            while "\n" in rx_buffer:
                line, rx_buffer = rx_buffer.split("\n", 1)
                line = line.strip()

                if line:
                    print(time.strftime("%H:%M:%S"), "GROUND_RX:", line)
                    lines.append(line)

        time.sleep(0.01)

    return lines

seq = 0

while True:
    print()
    print("========== POLL seq", seq, "==========")

    # Aynı REQ'i birkaç defa gönderiyoruz.
    # Çünkü tek paket bazen kaçabiliyor.
    for i in range(REQ_REPEAT):
        msg = f"REQ,{TARGET_ID},{seq}\r\n"
        written = ser.write(msg.encode("ascii"))
        ser.flush()

        print(time.strftime("%H:%M:%S"), "GROUND_TX:", msg.strip(), "bytes:", written)
        time.sleep(REQ_INTERVAL)

    print(time.strftime("%H:%M:%S"), "Ground silent, waiting for reply...")

    deadline = time.monotonic() + RESPONSE_TIMEOUT
    lines = read_lines_until(deadline)

    expected_prefix = f"N{TARGET_ID},{seq},"
    ok = False

    for line in lines:
        if line.startswith(expected_prefix):
            ok = True
            print(time.strftime("%H:%M:%S"), "RESULT: OK matched reply:", line)
            break

    if not ok:
        print(time.strftime("%H:%M:%S"), f"RESULT: TIMEOUT / NO VALID N{TARGET_ID},{seq},... REPLY")

    seq += 1
    time.sleep(1)