import sys
import io
import time
import base64
import threading
import queue
import tkinter as tk
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

# Load Waveshare's SX126x SPI driver
sys.path.insert(0, "/home/rhys/sx126x_lorawan_hat_code/python/lora")
from LoRaRF import SX126x

WIDTH = 640
HEIGHT = 480

B64_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
CHAR_TO_VAL = {ch: idx for idx, ch in enumerate(B64_CHARS)}

# Precomputed 623-byte .NET GDI+ 24-bit JPEG Headers for 8 Quality Levels (Q85..Q15)
JPEG_PREFIX_25 = base64.b64decode("/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAA==")
JPEG_SUFFIX_158 = base64.b64decode(
    "/8AAEQgAEABAAwEiAAIRAQMRAf/EAB8AAAEFAQEBAQEBAAAAAAAAAAABAgMEBQYHCAkKC//EALUQAAIBAwMCBAMFBQQEAAABfQECAwAEEQUSITFBBhNRYQcicRQygZGhCCNCscEVUtHwJDNicoIJChYXGBkaJSYnKCkqNDU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6g4SFhoeIiYqSk5SVlpeYmZqio6Slpqeoqaqys7S1tre4ubrCw8TFxsfIycrS09TV1tfY2drh4uPk5ebn6Onq8fLz9PX29/j5+v/EAB8BAAMBAQEBAQEBAQEAAAAAAAABAgMEBQYHCAkKC//EALURAAIBAgQEAwQHBQQEAAECdwABAgMRBAUhMQYSQVEHYXETIjKBCBRCkaGxwQkjM1LwFWJy0QoWJDThJfEXGBkaJicoKSo1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoKDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uLj5OXm5+jp6vLz9PX29/j5+v/aAAwDAQACEQMRAD8A"
)
JPEG_DQT_133_B64 = {
    0: "BQQEBAQDBQQEBAYFBQYIDQgIBwcIEAsMCQ0TEBQTEhASEhQXHRkUFhwWEhIaIxocHh8hISEUGSQnJCAmHSAhIP/bAEMBBQYGCAcIDwgIDyAVEhUgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIA==",
    1: "CAYGBwYFCAcHBwkJCAoMFA0MCwsMGRITDxQdGh8eHRocHCAkLicgIiwjHBwoNyksMDE0NDQfJzk9ODI8LjM0Mv/bAEMBCQkJDAsMGA0NGDIhHCEyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMg==",
    2: "DAgJCgkHDAoJCg0MDA4RHRMREBARIxkbFR0qJSwrKSUoKC40QjguMT8yKCg6Tjo/REdKS0otN1FXUUhWQklKR//bAEMBDA0NEQ8RIhMTIkcwKDBHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHR0dHRw==",
    3: "DwoLDQsJDw0MDREQDxEWJRgWFBQWLSAiGyU1Lzg3NC80MztCVUg7P1A/MzRKZEtQV1pfYF85R2hvZ1xuVV1fW//bAEMBEBERFhMWKxgYK1s9ND1bW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbW1tbWw==",
    4: "EgwOEA4LEhAPEBQTEhUbLR0bGRkbNygqIS1COkVEQDo/PkhRaFhITWJOPj9ae1xia290dnRGV4CJf3GIaHJ0cP/bAEMBExQUGxgbNR0dNXBLP0twcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcHBwcA==",
    5: "GBASFRIPGBUTFRoZGBwjOyYjICAjSDM2KztVS1pYVEtSUV5qh3NeZIBlUVJ2oHeAjJCXmZdbcaaypZOwh5SXkv/bAEMBGRoaIx8jRSYmRZJhUmGSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkg==",
    6: "IBYYHBgUIBwaHCQiICYwUDQwLCwwYkZKOlB0Znp4cmZwboCQuJyAiK6KbnCg2qKuvsTO0M58muLy4MjwuMrOxv/bAEMBIiQkMCowXjQ0XsaEcITGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxsbGxg==",
    7: "OScrMiskOTIuMkA9OURWj11WT09Wr32EaI/PttrWy7bIxOT////k8//2xMj////////////d///////////////bAEMBPUBAVktWqF1dqP/syOz//////////////////////////////////////////////////////////////////w==",
}
JPEG_HEADERS_64x16 = {
    q_idx: JPEG_PREFIX_25 + base64.b64decode(dqt_b64) + JPEG_SUFFIX_158
    for q_idx, dqt_b64 in JPEG_DQT_133_B64.items()
}

LEVEL_CONFIGS = {
    0: (12, 125000, 5, True,  "LEVEL 0 (SF12/125k)"),
    1: (11, 125000, 5, True,  "LEVEL 1 (SF11/125k)"),
    2: (10, 125000, 5, False, "LEVEL 2 (SF10/125k)"),
    3: (9,  250000, 5, False, "LEVEL 3 (SF9/250k)"),
    4: (8,  250000, 5, False, "LEVEL 4 (SF8/250k)"),
    5: (7,  250000, 5, False, "LEVEL 5 (SF7/250k Fast)"),
    6: (7,  500000, 5, False, "LEVEL 6 (SF7/500k Turbo)"),
    7: (6,  500000, 5, False, "LEVEL 7 (SF6/500k Max)"),
}
LEVEL_NAMES = {k: v[4] for k, v in LEVEL_CONFIGS.items()}

def compute_checksum_char(data_str):
    s = 0
    for ch in data_str:
        s = ((s << 1) ^ ord(ch)) & 0xFFFF
    return B64_CHARS[s & 0x3F]

def compute_checksum2(data_str):
    crc = 0xACE1
    for ch in data_str:
        crc ^= (ord(ch) << 4)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    v1 = (crc >> 6) & 0x3F
    v2 = crc & 0x3F
    return B64_CHARS[v1] + B64_CHARS[v2]

LoRa = SX126x()
orig_read = LoRa._readBytes
def safe_read(*args):
    for _ in range(5):
        res = orig_read(*args)
        if res and len(res) > 0:
            return res
        time.sleep(0.0005)
    return tuple([0] * 32)
LoRa._readBytes = safe_read

lora_lock = threading.Lock()
print("Initializing Waveshare SX1262 SPI HAT (CS=GPIO21, RST=18, BUSY=20, TXEN=6)...")
if not LoRa.begin(0, 0, 18, 20, -1, 6, -1):
    raise RuntimeError("Could not initialize Waveshare SX1262 SPI HAT!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setRxGain(LoRa.RX_GAIN_BOOSTED)

current_level = 5
last_rssi = 0.0
last_snr = 0.0

def set_lora_level(level_num):
    global current_level
    if level_num not in LEVEL_CONFIGS:
        return
    sf, bw, cr, ldro, name = LEVEL_CONFIGS[level_num]
    with lora_lock:
        current_level = level_num
        LoRa.setStandby(LoRa.STANDBY_RC)
        LoRa.setLoRaModulation(sf, bw, cr, ldro)
        LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 8, 255, True, False)
        LoRa.setSyncWord(0x1424)
        LoRa.request(LoRa.RX_CONTINUOUS)
        print(f"Waveshare SX1262 SPI HAT now listening on {name} @ 868.0 MHz!")

set_lora_level(current_level)
line_queue = queue.Queue()

def spi_lora_reader():
    global last_rssi, last_snr
    while True:
        try:
            pkt_text = None
            with lora_lock:
                irq = LoRa.getIrqStatus()
                if irq != 0:
                    LoRa.clearIrqStatus(0x03FF)
                    if (irq & LoRa.IRQ_RX_DONE) and not (irq & (LoRa.IRQ_CRC_ERR | LoRa.IRQ_HEADER_ERR)):
                        payload_len, buf_offset = LoRa.getRxBufferStatus()
                        if payload_len > 0:
                            raw_bytes = LoRa.readBuffer(buf_offset, payload_len)
                            pkt_text = bytes(raw_bytes).decode('utf-8', errors='ignore')
                            try:
                                rssi_pkt, snr_pkt, _ = LoRa.getPacketStatus()
                                last_rssi = rssi_pkt / -2.0
                                if snr_pkt > 127:
                                    snr_pkt -= 256
                                last_snr = snr_pkt / 4.0
                            except Exception:
                                pass
                    if irq & LoRa.IRQ_TIMEOUT:
                        LoRa.setRx(LoRa.RX_CONTINUOUS)

            if pkt_text:
                for line in pkt_text.splitlines():
                    line = line.strip()
                    if line:
                        line_queue.put(line)
            else:
                time.sleep(0.002)
        except Exception:
            time.sleep(0.01)

threading.Thread(target=spi_lora_reader, daemon=True).start()

root = tk.Tk()
root.title("SAR Drone - True 24-Bit JPEG 480p LoRa Receiver (16.7M Colors)")
root.configure(bg="#111827")

top_frame = tk.Frame(root, bg="#111827")
top_frame.pack(fill=tk.X, padx=12, pady=6)

status_label = tk.Label(
    top_frame,
    text=f"[{LEVEL_NAMES[current_level]}] Listening on 868.0 MHz SPI...",
    fg="#10B981",
    bg="#111827",
    font=("Monospace", 10, "bold")
)
status_label.pack(side=tk.LEFT)

def on_pi_level_change(val):
    lvl = int(val.split()[1])
    if lvl != current_level:
        threading.Thread(target=set_lora_level, args=(lvl,), daemon=True).start()

level_var = tk.StringVar(value=LEVEL_NAMES[current_level])
level_menu = tk.OptionMenu(top_frame, level_var, *[LEVEL_NAMES[i] for i in range(8)], command=on_pi_level_change)
level_menu.config(bg="#1F2937", fg="#FFFFFF", highlightthickness=0, font=("Monospace", 9))
level_menu.pack(side=tk.RIGHT)

canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="#000000", highlightthickness=2, highlightbackground="#374151")
canvas.pack(padx=12, pady=6)

# Native Tkinter PhotoImage supports binary PPM (P6) directly without needing python3-pil.imagetk!
tk_fb = tk.PhotoImage(width=WIDTH, height=HEIGHT)
img_item = canvas.create_image(0, 0, anchor=tk.NW, image=tk_fb)
scan_bar = canvas.create_rectangle(0, 0, WIDTH, 2, outline="#00FF66", width=1)

pil_fb = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))

TILES_X = WIDTH // 64
TILES_Y16 = HEIGHT // 16
TOTAL_TILES_300 = TILES_X * TILES_Y16
jpeg_tile_quality = [0] * TOTAL_TILES_300
locked_jpeg_tiles = set()
filled_jpeg_tiles = set()

jpg_full_total = 0
jpg_full_chunks = {}
seg_quality = [0] * (HEIGHT * 4)
locked_clean_rows = set()
filled_rows = set()

last_mode = "JPEG_TILE"
total_packets = 0
noisy_packets = 0
fb_dirty = False

def pil_to_tk_tile(pil_img, x0, y0):
    """Blits a 24-bit RGB PIL Image directly into tk.PhotoImage using fast binary PPM (P6) — zero ImageTk needed!"""
    w, h = pil_img.size
    ppm_bytes = f"P6 {w} {h} 255\n".encode("ascii") + pil_img.tobytes()
    temp_img = tk.PhotoImage(data=ppm_bytes)
    tk_fb.tk.call(tk_fb, "copy", temp_img, "-to", x0, y0)

def clear_screen():
    global pil_fb, fb_dirty, jpeg_tile_quality, locked_jpeg_tiles, filled_jpeg_tiles
    global jpg_full_total, jpg_full_chunks, seg_quality, locked_clean_rows, filled_rows, total_packets, noisy_packets
    pil_fb.paste((0, 0, 0), (0, 0, WIDTH, HEIGHT))
    tk_fb.put("#000000", to=(0, 0, WIDTH, HEIGHT))
    jpeg_tile_quality = [0] * TOTAL_TILES_300
    locked_jpeg_tiles.clear()
    filled_jpeg_tiles.clear()
    jpg_full_total = 0
    jpg_full_chunks.clear()
    seg_quality = [0] * (HEIGHT * 4)
    locked_clean_rows.clear()
    filled_rows.clear()
    total_packets = 0
    noisy_packets = 0
    fb_dirty = False

def process_jpeg_tile_packet(tx, ty, span_w, span_h, q_idx, chk_str, b64_body):
    global total_packets, noisy_packets, last_mode
    last_mode = "JPEG_TILE"
    if q_idx not in JPEG_HEADERS_64x16:
        return False

    is_clean = (compute_checksum2(b64_body) == chk_str)
    total_packets += 1
    if not is_clean:
        noisy_packets += 1

    try:
        pad_len = (-len(b64_body)) % 4
        b64_padded = b64_body + ("=" * pad_len) if pad_len else b64_body
        entropy_bytes = base64.b64decode(b64_padded)
        full_jpeg = JPEG_HEADERS_64x16[q_idx] + entropy_bytes + b"\xff\xd9"
        tile_img = Image.open(io.BytesIO(full_jpeg)).convert("RGB")
    except Exception:
        return False

    # Prepare one fast PPM PhotoImage for this 64x16 tile and copy it directly into tk_fb
    ppm_bytes = b"P6 64 16 255\n" + tile_img.tobytes()
    temp_tile = tk.PhotoImage(data=ppm_bytes)

    span_w, span_h = max(1, span_w), max(1, span_h)
    for dy in range(span_h):
        cur_ty = ty + dy
        if cur_ty >= TILES_Y16:
            break
        for dx in range(span_w):
            cur_tx = tx + dx
            if cur_tx >= TILES_X:
                break
            t_idx = cur_ty * TILES_X + cur_tx
            if is_clean and jpeg_tile_quality[t_idx] < 3:
                tk_fb.tk.call(tk_fb, "copy", temp_tile, "-to", cur_tx * 64, cur_ty * 16)
                filled_jpeg_tiles.add(t_idx)
                jpeg_tile_quality[t_idx] = 3
                locked_jpeg_tiles.add(t_idx)
            elif not is_clean and jpeg_tile_quality[t_idx] == 0:
                tk_fb.tk.call(tk_fb, "copy", temp_tile, "-to", cur_tx * 64, cur_ty * 16)
                filled_jpeg_tiles.add(t_idx)
                jpeg_tile_quality[t_idx] = 1

    canvas.coords(scan_bar, tx * 64, ty * 16, min(WIDTH, (tx + span_w) * 64), min(HEIGHT, (ty + span_h) * 16))
    return True

def process_full_jpeg_chunk(idx, total, chk_str, b64_chunk):
    global total_packets, noisy_packets, last_mode, fb_dirty, jpg_full_total, jpg_full_chunks
    last_mode = "JPEG_FULL"
    is_clean = (compute_checksum2(b64_chunk) == chk_str)
    total_packets += 1
    if not is_clean:
        noisy_packets += 1
        return False

    if total != jpg_full_total:
        jpg_full_total = total
        jpg_full_chunks.clear()
    jpg_full_chunks[idx] = b64_chunk

    contiguous = []
    for i in range(total):
        if i in jpg_full_chunks:
            contiguous.append(jpg_full_chunks[i])
        else:
            break

    if len(contiguous) >= 4 or len(jpg_full_chunks) == total:
        try:
            b64_joined = "".join(contiguous)
            pad_len = (-len(b64_joined)) % 4
            if pad_len:
                b64_joined += "=" * pad_len
            raw_jpg = base64.b64decode(b64_joined)
            if not raw_jpg.endswith(b"\xff\xd9"):
                raw_jpg += b"\xff\xd9"
            partial_img = Image.open(io.BytesIO(raw_jpg)).convert("RGB")
            pil_fb.paste(partial_img, (0, 0))
            fb_dirty = True
        except Exception:
            pass

    prog_y = int((len(jpg_full_chunks) / max(1, total)) * HEIGHT)
    canvas.coords(scan_bar, 0, min(HEIGHT - 2, prog_y), WIDTH, min(HEIGHT, prog_y + 2))
    return True

def decode_smooth_row(encoded, expected_width=WIDTH):
    if len(encoded) % 3 != 0:
        return None
    row_pixels = []
    prev_r = -1
    prev_g = -1
    prev_b = -1
    for i in range(0, len(encoded), 3):
        c_run, c_hi, c_lo = encoded[i], encoded[i + 1], encoded[i + 2]
        if c_run not in CHAR_TO_VAL or c_hi not in CHAR_TO_VAL or c_lo not in CHAR_TO_VAL:
            return None
        run_len = CHAR_TO_VAL[c_run] + 1
        v1, v2 = CHAR_TO_VAL[c_hi], CHAR_TO_VAL[c_lo]
        r = ((v1 >> 2) & 0x0F) * 17
        g = (((v1 & 0x03) << 2) | ((v2 >> 4) & 0x03)) * 17
        b = (v2 & 0x0F) * 17
        if prev_r < 0 or (abs(r - prev_r) + abs(g - prev_g) + abs(b - prev_b) > 34):
            row_pixels.extend([f"#{r:02x}{g:02x}{b:02x}"] * run_len)
        else:
            dr, dg, db = (r - prev_r) / run_len, (g - prev_g) / run_len, (b - prev_b) / run_len
            for k in range(1, run_len + 1):
                row_pixels.append(f"#{int(prev_r + dr * k):02x}{int(prev_g + dg * k):02x}{int(prev_b + db * k):02x}")
        prev_r, prev_g, prev_b = r, g, b
    if len(row_pixels) != expected_width:
        return None
    return "{" + " ".join(row_pixels) + "}"

def process_legacy_scanline(y, span_h, chk_str, encoded):
    global total_packets, noisy_packets, last_mode
    last_mode = "SCAN"
    row_str = decode_smooth_row(encoded, WIDTH)
    if row_str is None:
        return False
    is_clean = (compute_checksum2(encoded) == chk_str) if len(chk_str) == 2 else (compute_checksum_char(encoded) == chk_str)
    total_packets += 1
    if not is_clean:
        noisy_packets += 1
    for dy in range(max(1, span_h)):
        ty = y + dy
        if ty >= HEIGHT:
            break
        if is_clean and seg_quality[ty * 4] < 3:
            tk_fb.put(row_str, to=(0, ty))
            filled_rows.add(ty)
            for q in range(4):
                seg_quality[ty * 4 + q] = 3
            locked_clean_rows.add(ty)
        elif not is_clean and seg_quality[ty * 4] == 0:
            tk_fb.put(row_str, to=(0, ty))
            filled_rows.add(ty)
            for q in range(4):
                seg_quality[ty * 4 + q] = 1
    canvas.coords(scan_bar, 0, y, WIDTH, min(HEIGHT, y + max(2, span_h)))
    return True

def update_gui():
    global fb_dirty
    for _ in range(35):
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
                    level_var.set(LEVEL_NAMES[new_lvl])
                    threading.Thread(target=set_lora_level, args=(new_lvl,), daemon=True).start()
            except ValueError:
                pass
            continue

        if raw.startswith("J:"):
            parts = raw.split(":")
            if len(parts) == 8:
                try:
                    process_jpeg_tile_packet(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5]), parts[6], parts[7])
                except ValueError:
                    pass
            continue

        if raw.startswith("F:"):
            parts = raw.split(":")
            if len(parts) == 5:
                try:
                    process_full_jpeg_chunk(int(parts[1]), int(parts[2]), parts[3], parts[4])
                except ValueError:
                    pass
            continue

        if raw.startswith("S:"):
            parts = raw.split(":")
            if len(parts) == 5:
                try:
                    process_legacy_scanline(int(parts[1]), int(parts[2]), parts[3], parts[4])
                except ValueError:
                    pass
            continue

    if fb_dirty:
        pil_to_tk_tile(pil_fb, 0, 0)
        fb_dirty = False

    rf_info = f"RSSI:{last_rssi:.0f}dBm" if total_packets > 0 else "868MHz"
    if last_mode == "JPEG_TILE":
        fill_pct = (len(filled_jpeg_tiles) / TOTAL_TILES_300) * 100
        clean_pct = (len(locked_jpeg_tiles) / TOTAL_TILES_300) * 100
        status_label.config(
            text=f"[{LEVEL_NAMES.get(current_level, '')} | {rf_info}] 24-Bit JPEG Tiles: {fill_pct:.0f}% | Clean: {len(locked_jpeg_tiles)}/{TOTAL_TILES_300} ({clean_pct:.0f}%) | Static: {noisy_packets}"
        )
    elif last_mode == "JPEG_FULL":
        pct = (len(jpg_full_chunks) / max(1, jpg_full_total)) * 100 if jpg_full_total > 0 else 0
        status_label.config(
            text=f"[{LEVEL_NAMES.get(current_level, '')} | {rf_info}] 24-Bit Full JPEG: {len(jpg_full_chunks)}/{jpg_full_total} ({pct:.0f}%) | Static: {noisy_packets}"
        )
    else:
        fill_pct = (len(filled_rows) / HEIGHT) * 100
        clean_pct = (len(locked_clean_rows) / HEIGHT) * 100
        status_label.config(
            text=f"[{LEVEL_NAMES.get(current_level, '')} | {rf_info}] 480p Scan: {fill_pct:.0f}% | Clean: {len(locked_clean_rows)}/{HEIGHT} ({clean_pct:.0f}%) | Static: {noisy_packets}"
        )
    root.after(15, update_gui)

root.after(30, update_gui)
root.mainloop()
