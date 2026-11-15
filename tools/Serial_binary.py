import serial
import csv
from datetime import datetime
from pathlib import Path
from log_protocol import saved_log_frame, saved_log_magic, decode_saved_values

# ===== 設定 =====
PORT = "COM6"
BAUD = 460800          # マウスの USART1 設定に合わせる
MOUSE = "type8a_MPQ"

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

frame = None  # HEADERで新旧の列数と型を識別する。
header = None

count = 0
drop  = 0
header_written = False
binary_mode = False

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
                    frame = saved_log_frame(header)
                    writer.writerow(header)
                    header_written = True
                    print("HEADER written",header)
                    continue

                if line == "BINARY":
                    if frame is None:
                        raise ValueError('HEADERが必要です。log mode0で再取得してください。')
                    print("=== Binary mode start ===")
                    binary_mode = True
                    continue

                if line == "END":
                    print("END detected")
                    break

                continue

            # ===== BINARYモード =====
            raw = ser.read(frame.size)
            if len(raw) != frame.size:
                continue

            magic, idx, *half_data = frame.unpack(raw)

            # --- Binary END ---
            if magic == MAGIC_END:
                print("=== Binary END frame detected ===")
                binary_mode = False
                continue

            # --- 不正フレーム ---
            if magic != saved_log_magic(header):
                drop += 1
                continue

            row = [idx] + decode_saved_values(header, half_data)
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
