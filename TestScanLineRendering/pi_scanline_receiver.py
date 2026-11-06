import serial
import time
import threading
import queue
import tkinter as tk

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

WIDTH = 120
HEIGHT = 90
SCALE = 4

B64_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

# Precompute 64-color RGB222 lookup table (#RRGGBB)
COLOR_LUT = {}
for idx, ch in enumerate(B64_CHARS):
    r = ((idx >> 4) & 0x03) * 85
    g = ((idx >> 2) & 0x03) * 85
    b = (idx & 0x03) * 85
    COLOR_LUT[ch] = f"#{r:02x}{g:02x}{b:02x}"

LEVEL_NAMES = {
    0: "LEVEL 0 (336 bps)",
    1: "LEVEL 1 (1,075 bps)",
    2: "LEVEL 2 (2,149 bps)",
    3: "LEVEL 3 (6,250 bps)",
    4: "LEVEL 4 (10,417 bps)",
    5: "LEVEL 5 (18,230 bps)",
    6: "LEVEL 6 (37,500 bps)",
    7: "LEVEL 7 (62,500 bps)",
}

lora = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2.0)
lora_lock = threading.Lock()

def set_lora_level(level_num):
    with lora_lock:
        print(f"Switching Pi DX-LR22 to LEVEL {level_num}...")
        lora.reset_input_buffer()
        lora.write(b"+++\r\n")
        time.sleep(0.6)
        lora.read_all()
        lora.write(f"AT+LEVEL{level_num}\r\n".encode('utf-8'))
        time.sleep(0.5)
        lora.read_all()
        lora.write(b"AT+RESET\r\n")
        time.sleep(0.8)
        lora.reset_input_buffer()
        print(f"Pi DX-LR22 now on {LEVEL_NAMES.get(level_num, level_num)}!")

# Start on LEVEL 2 (2,149 bps) to match Laptop Studio default
time.sleep(0.3)
set_lora_level(2)

line_queue = queue.Queue()

def serial_reader():
    buffer = ""
    while True:
        try:
            with lora_lock:
                waiting = lora.in_waiting
                chunk = lora.read(waiting if waiting > 0 else 1).decode('utf-8', errors='ignore')
            if chunk:
                buffer += chunk
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        line_queue.put(line)
            else:
                time.sleep(0.01)
        except Exception:
            time.sleep(0.05)

threading.Thread(target=serial_reader, daemon=True).start()

# Setup Tkinter GUI
root = tk.Tk()
root.title("SAR Drone - Live Progressive Scanline Receiver")
root.configure(bg="#111827")

current_level = 2
status_label = tk.Label(
    root,
    text=f"[{LEVEL_NAMES[current_level]}] Waiting for scanlines... (0/90 lines)",
    fg="#10B981",
    bg="#111827",
    font=("Monospace", 10, "bold"),
    pady=8
)
status_label.pack()

canvas = tk.Canvas(
    root,
    width=WIDTH * SCALE,
    height=HEIGHT * SCALE,
    bg="#000000",
    highlightthickness=2,
    highlightbackground="#374151"
)
canvas.pack(padx=12, pady=8)

img = tk.PhotoImage(width=WIDTH * SCALE, height=HEIGHT * SCALE)
canvas.create_image(0, 0, anchor=tk.NW, image=img)
scan_bar = canvas.create_rectangle(0, 0, WIDTH * SCALE, SCALE, outline="#00FF66", width=2)

received_rows = set()
total_packets = 0
dropped_packets = 0

def clear_screen():
    global received_rows, total_packets, dropped_packets
    black_row = " ".join(["#000000"] * (WIDTH * SCALE))
    block = " ".join(["{" + black_row + "}"] * (HEIGHT * SCALE))
    img.put(block, to=(0, 0))
    received_rows.clear()
    total_packets = 0
    dropped_packets = 0

def update_gui():
    global total_packets, dropped_packets, current_level

    for _ in range(5):
        if line_queue.empty():
            break
        raw = line_queue.get_nowait()

        if raw == "CMD:CLEAR":
            clear_screen()
            continue

        if raw.startswith("CMD:LEVEL:"):
            try:
                new_lvl = int(raw.split(":")[2])
                if 0 <= new_lvl <= 7 and new_lvl != current_level:
                    current_level = new_lvl
                    threading.Thread(target=set_lora_level, args=(new_lvl,), daemon=True).start()
            except ValueError:
                pass
            continue

        if raw.startswith("L:"):
            parts = raw.split(":")
            if len(parts) == 3:
                try:
                    y = int(parts[1])
                    pixels = parts[2]
                    if 0 <= y < HEIGHT and len(pixels) == WIDTH:
                        scaled_row = " ".join(
                            " ".join([COLOR_LUT.get(c, "#000000")] * SCALE) for c in pixels
                        )
                        block = " ".join(["{" + scaled_row + "}"] * SCALE)
                        img.put(block, to=(0, y * SCALE))

                        received_rows.add(y)
                        total_packets += 1
                        canvas.coords(scan_bar, 0, y * SCALE, WIDTH * SCALE, (y + 1) * SCALE)
                    else:
                        dropped_packets += 1
                except ValueError:
                    dropped_packets += 1
            else:
                dropped_packets += 1

    pct = (len(received_rows) / HEIGHT) * 100
    status_label.config(
        text=f"[{LEVEL_NAMES.get(current_level, '')}] Build: {len(received_rows)}/{HEIGHT} ({pct:.0f}%) | Good: {total_packets} | Bad: {dropped_packets}"
    )
    root.after(25, update_gui)

root.after(50, update_gui)
root.mainloop()
