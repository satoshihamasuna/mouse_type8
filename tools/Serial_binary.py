import serial
import csv
import struct
import numpy as np
from datetime import datetime
from pathlib import Path

# ===== 設定 =====
PORT = "COM6"
BAUD = 115200          # ★必ず上げる
MOUSE = "type8a_MPQ"

LOG_DATA_NUM = 51     # ← STM32側と一致させる
MAGIC = 0xA55A
MAGIC_END = 0xFFFF

# =================

LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

fname = datetime.now().strftime("%Y%m%d_%H%M_"+MOUSE+".csv")
fpath = LOG_DIR / fname

ser = serial.Serial(PORT, BAUD, timeout=1)

print("="*60)
print(" Serial Logging Start")
print(f" Port : {PORT}")
print(f" Baud : {BAUD}")
print(f" File : {fpath}")
print("="*60)

FRAME_FMT  = "<HH" + "H"*LOG_DATA_NUM
FRAME_SIZE = struct.calcsize(FRAME_FMT)

count = 0
drop  = 0
header_written = False
binary_mode = False

def half_to_float(h):
    return np.frombuffer(np.uint16(h).tobytes(), dtype=np.float16)[0].astype(np.float32)

with open(fpath, "w", newline="") as f:
    writer = csv.writer(f)

    try:
        while True:

            # ===== ASCIIモード =====
            if not binary_mode:
                line = ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                #print("ASCII:", line)

                if line.startswith("HEADER,"):
                    header = line.split(",")[1:]
                    writer.writerow(header)
                    header_written = True
                    print("HEADER written",header)
                    continue

                if line == "BINARY":
                    print("=== Binary mode start ===")
                    binary_mode = True
                    continue

                if line == "END":
                    print("END detected")
                    break

                continue

            # ===== BINARYモード =====
            raw = ser.read(FRAME_SIZE)
            if len(raw) != FRAME_SIZE:
                continue

            magic, idx, *half_data = struct.unpack(FRAME_FMT, raw)

            # --- Binary END ---
            if magic == MAGIC_END:
                print("=== Binary END frame detected ===")
                binary_mode = False
                continue

            # --- 不正フレーム ---
            if magic != MAGIC:
                drop += 1
                continue

            row = [idx] + [half_to_float(x) for x in half_data]
            writer.writerow(row)

            count += 1
            if count % 50 == 0:
                print(f"write: {count}")

    except KeyboardInterrupt:
        print("\nLogging stopped by user")

    finally:
        ser.close()
        print("="*60)
        print(f"Total write: {count}")
        print(f"Total drop : {drop}")
        print("Serial port closed")
        print("="*60)
