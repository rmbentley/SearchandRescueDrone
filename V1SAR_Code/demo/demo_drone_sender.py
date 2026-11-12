#!/usr/bin/env python3
"""
========================================================================================
Search & Rescue (SAR) Drone System — Mission Control & Drone AI Camera Simulator (DEMO)
========================================================================================
This script emulates the airborne Raspberry Pi 5 + Sony IMX500 AI Camera +
Waveshare SX1262 LoRa SPI HAT transmitter.
It runs WITHOUT any hardware:
  1. Simulates the Sony IMX500 2028x1520 @ 30 FPS optical sensor:
     - Uses a rich synthetic aerial SAR terrain scene (or custom loaded image)
     - Interactive Target: Click anywhere on the Wide view to move the lost person!
     - Autonomous Flight Simulation: Damped gimbal tracking and drift
  2. Runs the 2-Stage Target Lock State Machine:
     - Stage 1: WIDE_SEARCH (3 consecutive frames >= Wide Trigger)
     - Stage 2: ZOOM_CONFIRM (Tracking scope zooms in, 3 frames >= Confirm Lock)
     - Automatic Lock & Broadcast Trigger
  3. Composes Multi-Resolution Snapshots (320x240 to 1280x960):
     - Automatically places the Zoomed Scope PiP in the corner OPPOSITE the target!
     - Telemetry bar with GPS coordinates, confidence %, time, and resolution
  4. Encodes True 64x16 24-bit JPEG Picture Blocks:
     - Shared quantization header (JH:)
     - Per-block BoxBlur(1) safety guard (<=184 characters)
     - Target-First Radial Block Sorting (candidate person renders first!)
  5. Virtual LoRa RF Physical Layer:
     - Broadcasts over local UDP (port 9868)
     - Matched LoRa level modulation presets (L0 to L7) with authentic inter-packet gaps
     - Interactive Packet Drop Slider (0% to 30%) to simulate real-world RF packet loss
     - Interactive RF Signal Power Slider (-45 dBm to -125 dBm)
========================================================================================
"""

import sys
import os
import math
import random
import socket
import threading
import time
import datetime
import io
import base64
import queue
import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageDraw, ImageFont, ImageTk, ImageFilter

DEFAULT_LORA_UDP_PORT = 9868
DEFAULT_SENDER_LISTEN_PORT = 9869

# 12-bit Base64 Table
B64_TABLE = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

def compute_checksum2(data_str):
    crc = 0xACE1
    for ch in data_str:
        crc ^= (ord(ch) << 4)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    c1 = B64_TABLE[(crc >> 6) & 0x3F]
    c2 = B64_TABLE[crc & 0x3F]
    return c1 + c2

def split_jpeg_header_and_scan(jpg_bytes):
    idx = 0
    n = len(jpg_bytes)
    while idx < n - 4:
        if jpg_bytes[idx] == 0xFF and jpg_bytes[idx + 1] == 0xDA:
            sos_len = (jpg_bytes[idx + 2] << 8) | jpg_bytes[idx + 3]
            raw_split = idx + 2 + sos_len
            split_pos = (raw_split // 3) * 3
            return jpg_bytes[:split_pos], jpg_bytes[split_pos:]
        idx += 1
    return b"", jpg_bytes

# --- Matched 8-Level LoRa Modulation Presets ---
LEVEL_CONFIGS = [
    # Level, SF, BW_kHz, CR, Gap_ms, Name
    (0, 12, 125, "4/5", 1450, "L0 · SF12 / 125k (Extreme NLOS)"),
    (1, 11, 125, "4/5",  780, "L1 · SF11 / 125k (Very Long Range)"),
    (2, 10, 125, "4/5",  410, "L2 · SF10 / 125k (Long Range)"),
    (3,  9, 250, "4/5",  175, "L3 · SF9 / 250k (Medium Range)"),
    (4,  8, 250, "4/5",  110, "L4 · SF8 / 250k (Medium-Fast)"),
    (5,  7, 250, "4/5",   75, "L5 · SF7 / 250k (Standard Fast)"),
    (6,  7, 500, "4/5",   55, "L6 · SF7 / 500k (Turbo Line-Of-Sight)"),
    (7,  6, 500, "4/5",   45, "L7 · SF6 / 500k (Maximum Speed)"),
]

# Resolution Presets: Code, Label, Width, Height, TileCols, TileRows, TotalTiles
RES_PRESETS = [
    ("RES_320",  "320×240 (75 blk)",    320,  240,  5, 15,   75),
    ("RES_640",  "640×480 (300 blk)",   640,  480, 10, 30,  300),
    ("RES_960",  "960×720 (675 blk)",   960,  720, 15, 45,  675),
    ("RES_1280", "1280×960 (1200 blk)", 1280, 960, 20, 60, 1200),
]

AI_MODELS = [
    ("IMX500 SSD",     "SSD MobileNetV2 (Sony NPU)"),
    ("EfficientDet",   "EfficientDet-Lite0"),
    ("YOLOv8n",        "YOLOv8n (SAR Tuned)"),
    ("NanoDet",        "NanoDet-Plus"),
]

TX_MODES = [
    ("SCAN_TARGET", "Target-First Blocks", "Radially Outward from Person"),
    ("SCAN_ROWS",   "Row-Order Blocks",   "Top-to-Bottom 64×16 Rows"),
    ("2D_FULL",     "Full-Frame Packets", "2D Progressive JPEG Packets"),
]

# --- Global Simulator State ---
BINNED_W = 2028
BINNED_H = 1520
TILE_W = 64
TILE_H = 16

TX_W = 640
TX_H = 480
active_res_code = "RES_640"
active_model_name = "IMX500 SSD"
active_tx_mode = "SCAN_TARGET"
current_level = 5

tx_active = False
tx_session_id = 0
tx_progress_idx = 0
tx_progress_total = 0
tx_pass = 1
capture_id = 0
heartbeat_seq = 0

# Target & Optical Simulator Variables
target_x = 1180.0
target_y = 840.0
target_w = 42.0
target_h = 58.0
target_detected = True
target_conf = 88.5
target_drift_vx = 0.4
target_drift_vy = 0.2

# Damped tracking gimbal
gimbal_cx = 1014.0
gimbal_cy = 760.0
gimbal_zoom = 2.5

# 2-Stage Lock FSM
zoom_state = "WIDE_SEARCH"
cand_lock_count = 0
confirm_lock_count = 0
last_auto_tx_time = time.time() + 60.0

# RF Simulation Controls
sim_packet_loss_pct = 0.0  # Slider: 0% to 30%
sim_rssi_dbm = -68.0       # Slider: -45 to -125 dBm
sim_snr_db = 8.5

gps_latitude = "-33.8688"
gps_longitude = "151.2093"
gps_altitude = "84m"

# Socket configuration
target_receiver_host = "127.0.0.1"
target_receiver_port = DEFAULT_LORA_UDP_PORT
udp_tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Base high-res terrain image
base_sensor_image = None
wide_base_small = None          # Pre-scaled copy of the sensor frame for the wide viewport (fast redraw)

WIDE_DISP_W, WIDE_DISP_H = 460, 345    # 4:3, same aspect as the 2028x1520 sensor
SCOPE_DISP_W, SCOPE_DISP_H = 460, 245

# Background threads (UDP listener, broadcast worker) must NEVER touch Tkinter directly.
# They post commands here and the Tk main loop executes them.
ui_cmd_queue = queue.Queue()

last_hb_time = 0.0
HB_INTERVAL_S = 2.5
AUTO_TX_COOLDOWN_S = 30.0

def lora_time_on_air_s(n_bytes, level):
    """Semtech SX126x LoRa time-on-air (explicit header, CRC on, CR 4/5, 8-symbol preamble)."""
    _, sf, bw_khz, _, _, _ = LEVEL_CONFIGS[level]
    ts = (2 ** sf) / (bw_khz * 1000.0)
    de = 1 if ts > 0.016 else 0
    payload_symb = 8 + max(math.ceil((8 * n_bytes - 4 * sf + 28 + 16) / (4.0 * (sf - 2 * de))) * 5, 0)
    return (8 + 4.25) * ts + payload_symb * ts

def lora_tx(line_str):
    """Send one packet over the virtual LoRa RF channel."""
    virtual_lora_send(line_str)

def init_base_terrain_image():
    global base_sensor_image
    # Look for sample_sar_aerial.jpg in demo directory
    sample_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sample_sar_aerial.jpg")
    if os.path.isfile(sample_path):
        try:
            base_sensor_image = Image.open(sample_path).convert("RGB")
            if base_sensor_image.size != (BINNED_W, BINNED_H):
                base_sensor_image = base_sensor_image.resize((BINNED_W, BINNED_H), Image.Resampling.BILINEAR)
            refresh_wide_base()
            return
        except Exception:
            pass

    # Fallback procedural generation
    base_sensor_image = Image.new("RGB", (BINNED_W, BINNED_H), (74, 88, 52))
    draw = ImageDraw.Draw(base_sensor_image)
    for y in range(0, BINNED_H, 10):
        c = int(68 + math.sin(y * 0.005) * 15)
        draw.rectangle([0, y, BINNED_W, y + 10], fill=(c, c + 15, c - 20))
    for _ in range(600):
        tx = random.randint(10, BINNED_W - 10)
        ty = random.randint(10, BINNED_H - 10)
        tr = random.randint(20, 60)
        draw.ellipse([tx, ty, tx + tr, ty + tr], fill=(36, 68, 32))
    refresh_wide_base()

def refresh_wide_base():
    global wide_base_small
    wide_base_small = base_sensor_image.resize((WIDE_DISP_W, WIDE_DISP_H), Image.Resampling.BILINEAR)

def draw_person(draw, hx, hy, s=1.0):
    """Draws the lost hiker (top-down) at sensor coords (hx, hy) scaled by s."""
    def r(v): return int(v * s)
    draw.ellipse([hx + r(4), hy + r(8), hx + r(28), hy + r(22)], fill=(20, 28, 18))
    draw.ellipse([hx, hy, hx + max(2, r(22)), hy + max(2, r(20))], fill=(245, 85, 20))
    draw.ellipse([hx + r(6), hy - r(6), hx + max(1, r(16)), hy + r(4)], fill=(235, 200, 40))
    draw.rectangle([hx + r(2), hy + r(16), hx + r(10), hy + r(26)], fill=(40, 60, 95))
    draw.rectangle([hx + r(12), hy + r(16), hx + r(20), hy + r(26)], fill=(40, 60, 95))

def virtual_lora_send(line_str):
    global sim_packet_loss_pct, sim_rssi_dbm, sim_snr_db
    # Simulated packet drop check (only on block/chunk data packets, never on CFG/H/CMD)
    if line_str.startswith("J") or line_str.startswith("F:"):
        if sim_packet_loss_pct > 0.0:
            if random.random() * 100.0 < sim_packet_loss_pct:
                # Intentionally drop packet to simulate RF interference
                return

    # Add virtual RF telemetry header [RSSI=...;SNR=...]
    rf_hdr = f"[RSSI={sim_rssi_dbm:.1f};SNR={sim_snr_db:.1f}]"
    payload = (rf_hdr + line_str + "\n").encode("ascii")
    try:
        udp_tx_socket.sendto(payload, (target_receiver_host, target_receiver_port))
    except Exception:
        pass

def broadcast_config_to_receiver():
    global current_level, active_tx_mode, active_model_name, TX_W, TX_H
    cfg_pkt = f"CFG:{current_level}:{active_tx_mode}:{active_model_name}:58:{TX_W}:{TX_H}"
    for _ in range(3):
        virtual_lora_send(cfg_pkt)
        time.sleep(0.04)

# --- Opposite-Corner Picture-in-Picture Snapshot Composer ---
def build_tx_snapshot_opposite_pip(sensor_pil, bx, by, bw, bh, conf_pct):
    global TX_W, TX_H
    # 1. Downscale full sensor frame to transmission resolution
    snap = sensor_pil.resize((TX_W, TX_H), Image.Resampling.BILINEAR)
    draw = ImageDraw.Draw(snap)

    scale_x = TX_W / float(BINNED_W)
    scale_y = TX_H / float(BINNED_H)

    # 2. Draw target bounding box and crosshairs on wide overview
    sx = int(bx * scale_x)
    sy = int(by * scale_y)
    sw = max(8, int(bw * scale_x))
    sh = max(12, int(bh * scale_y))

    draw.rectangle([sx, sy, sx + sw, sy + sh], outline=(239, 68, 68), width=2)
    # Crosshairs
    cx = sx + sw // 2
    cy = sy + sh // 2
    draw.line([(cx - 8, cy), (cx + 8, cy)], fill=(239, 68, 68), width=1)
    draw.line([(cx, cy - 8), (cx, cy + 8)], fill=(239, 68, 68), width=1)

    # 3. Determine target quadrant and choose diagonally opposite corner for PiP
    is_right = (cx >= TX_W * 0.5)
    is_bottom = (cy >= TX_H * 0.5)

    pip_w = int(TX_W * 0.32)
    pip_h = int(TX_H * 0.32)
    margin = 8

    if is_right and is_bottom:
        # Target in Bottom-Right -> Place PiP in Top-Left
        pip_x0, pip_y0 = margin, margin
    elif not is_right and is_bottom:
        # Target in Bottom-Left -> Place PiP in Top-Right
        pip_x0, pip_y0 = TX_W - pip_w - margin, margin
    elif is_right and not is_bottom:
        # Target in Top-Right -> Place PiP in Bottom-Left
        pip_x0, pip_y0 = margin, TX_H - pip_h - margin - 22
    else:
        # Target in Top-Left -> Place PiP in Bottom-Right
        pip_x0, pip_y0 = TX_W - pip_w - margin, TX_H - pip_h - margin - 22

    # 4. Crop zoomed scope region directly from the full 2028x1520 frame
    crop_w = int(bw * 2.8)
    crop_h = int(bh * 2.8)
    crop_x0 = max(0, min(BINNED_W - crop_w, int(bx + bw * 0.5 - crop_w * 0.5)))
    crop_y0 = max(0, min(BINNED_H - crop_h, int(by + bh * 0.5 - crop_h * 0.5)))
    scope_crop = sensor_pil.crop((crop_x0, crop_y0, crop_x0 + crop_w, crop_y0 + crop_h))
    scope_resized = scope_crop.resize((pip_w, pip_h), Image.Resampling.BILINEAR)

    # Paste zoomed PiP
    snap.paste(scope_resized, (pip_x0, pip_y0))
    draw.rectangle([pip_x0, pip_y0, pip_x0 + pip_w, pip_y0 + pip_h], outline=(37, 99, 235), width=2)
    draw.rectangle([pip_x0, pip_y0, pip_x0 + pip_w, pip_y0 + 16], fill=(17, 19, 23))
    draw.text((pip_x0 + 4, pip_y0 + 2), f"SCOPE 2.8X · {conf_pct:.0f}%", fill=(243, 244, 246))

    # 5. Bottom Telemetry Bar
    bar_h = 20
    draw.rectangle([0, TX_H - bar_h, TX_W, TX_H], fill=(17, 19, 23))
    tele_txt = f"SAR CONF: {conf_pct:.0f}% | {TX_W}x{TX_H} | GPS: {gps_latitude},{gps_longitude} ({gps_altitude}) | {active_model_name}"
    draw.text((8, TX_H - bar_h + 3), tele_txt, fill=(243, 244, 246))

    return snap

# --- Background LoRa Transmission Worker ---
def broadcast_worker(pil_snapshot, max_conf, num_passes, my_session, target_box):
    global tx_active, tx_progress_idx, tx_progress_total, tx_pass, capture_id
    tx_active = True
    capture_id += 1
    cur_id = capture_id
    snap_w, snap_h = pil_snapshot.size

    is_scanline = (active_tx_mode in ("SCAN_TARGET", "SCAN_ROWS"))
    target_first = (active_tx_mode == "SCAN_TARGET")

    pkt_gap_s = max(0.045, LEVEL_CONFIGS[current_level][4] / 1000.0)
    short_time = datetime.datetime.now().strftime("%H:%M:%S")
    gps_str = f"{gps_latitude},{gps_longitude},{gps_altitude}"

    try:
        if not is_scanline:
            # Mode: 2D Full Frame JPEG
            buf = io.BytesIO()
            pil_snapshot.save(buf, format="JPEG", quality=60, optimize=True)
            jpg_bytes = buf.getvalue()
            full_b64 = base64.b64encode(jpg_bytes).decode("ascii").rstrip("=")
            chunk_size = 188
            chunks = [full_b64[i:i + chunk_size] for i in range(0, len(full_b64), chunk_size)]
            tx_progress_total = len(chunks)

            det_pkt = f"D:{cur_id}:{max_conf:.1f}:1:{len(chunks)}:60:{snap_w}:{snap_h}"
            meta_pkt = f"M:{cur_id}:{active_tx_mode}:{short_time}:{gps_str.replace(':', ';')}:[Demo {snap_w}x{snap_h}]"

            for _ in range(2):
                if not tx_active or tx_session_id != my_session: return
                virtual_lora_send(det_pkt)
                time.sleep(0.04)
                virtual_lora_send(meta_pkt)
                time.sleep(0.04)

            for p in range(1, num_passes + 1):
                tx_pass = p
                for idx, chunk in enumerate(chunks):
                    if not tx_active or tx_session_id != my_session: return
                    tx_progress_idx = idx + 1
                    chk2 = compute_checksum2(chunk)
                    pkt = f"F:{idx}:{len(chunks)}:{chk2}:{chunk}"
                    virtual_lora_send(pkt)
                    time.sleep(pkt_gap_s)
        else:
            # Mode: 64x16 Picture Blocks
            t_cols = snap_w // TILE_W
            t_rows = snap_h // TILE_H
            total_tiles = t_cols * t_rows
            tx_progress_total = total_tiles

            tile_q = 60
            while True:
                tile_packets = []
                common_hdr_b64 = ""
                max_b64_len = 0
                for r in range(t_rows):
                    for c in range(t_cols):
                        x0, y0 = c * TILE_W, r * TILE_H
                        tile_img = pil_snapshot.crop((x0, y0, x0 + TILE_W, y0 + TILE_H))
                        for _blur in range(4):
                            tbuf = io.BytesIO()
                            tile_img.save(tbuf, format="JPEG", quality=tile_q, subsampling=2)
                            hdr_b, scan_b = split_jpeg_header_and_scan(tbuf.getvalue())
                            if not common_hdr_b64 and hdr_b:
                                common_hdr_b64 = base64.b64encode(hdr_b).decode("ascii")
                            scan_b64 = base64.b64encode(scan_b).decode("ascii").rstrip("=")
                            if len(scan_b64) <= 184:
                                break
                            tile_img = tile_img.filter(ImageFilter.BoxBlur(1))
                        if len(scan_b64) > max_b64_len:
                            max_b64_len = len(scan_b64)
                        tile_packets.append((c, r, scan_b64))
                if max_b64_len <= 184 or tile_q <= 30:
                    break
                tile_q = max(30, tile_q - 5)

            # Target-First Radial Sort
            if target_first and target_box is not None:
                bx, by, bw, bh = target_box
                tcx = ((bx + bw * 0.5) / BINNED_W) * t_cols
                tcy = ((by + bh * 0.5) / BINNED_H) * t_rows
                tile_packets.sort(key=lambda item: (item[0] + 0.5 - tcx) ** 2 + (item[1] + 0.5 - tcy) ** 2)

            det_pkt = f"D:{cur_id}:{max_conf:.1f}:1:{total_tiles}:{tile_q}:{snap_w}:{snap_h}"
            meta_pkt = f"M:{cur_id}:{active_tx_mode}:{short_time}:{gps_str.replace(':', ';')}:[Demo {snap_w}x{snap_h}]"
            hdr_chunks = [common_hdr_b64[i:i + 180] for i in range(0, len(common_hdr_b64), 180)]

            for _ in range(2):
                if not tx_active or tx_session_id != my_session: return
                virtual_lora_send(det_pkt)
                time.sleep(0.04)
                virtual_lora_send(meta_pkt)
                time.sleep(0.04)
                for h_idx, h_part in enumerate(hdr_chunks):
                    virtual_lora_send(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                    time.sleep(0.03)

            for p in range(1, num_passes + 1):
                tx_pass = p
                if p > 1:
                    # Pass 2 Rescan re-announces headers and sends missed repair blocks!
                    virtual_lora_send(det_pkt)
                    time.sleep(0.04)
                    virtual_lora_send(meta_pkt)
                    time.sleep(0.04)
                    for h_idx, h_part in enumerate(hdr_chunks):
                        virtual_lora_send(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                        time.sleep(0.03)

                pass_gap = pkt_gap_s + (0.015 if p > 1 else 0.0)
                for idx, (c, r, scan_b64) in enumerate(tile_packets):
                    if not tx_active or tx_session_id != my_session: return
                    tx_progress_idx = idx + 1
                    if idx > 0 and (idx % 35 == 0):
                        virtual_lora_send(det_pkt)
                        time.sleep(pass_gap)
                        for h_idx, h_part in enumerate(hdr_chunks):
                            virtual_lora_send(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                            time.sleep(pass_gap)

                    chk2 = compute_checksum2(scan_b64)
                    pkt = f"J{t_cols}:{c}:{r}:{chk2}:{scan_b64}"
                    virtual_lora_send(pkt)
                    time.sleep(pass_gap)

    except Exception as e:
        print(f"[Broadcast Error] {e}")
    finally:
        if tx_session_id == my_session:
            tx_active = False

def trigger_broadcast_now():
    global tx_session_id, tx_active
    tx_session_id += 1
    my_session = tx_session_id
    tx_active = True

    if btn_transmit:
        btn_transmit.config(text="● BROADCASTING...", bg=C_GREEN, fg="#FFFFFF")
    if lbl_tx_progress:
        lbl_tx_progress.config(text="PREPARING LORA BROADCAST (ENCODING)...", fg=C_AMBER)
    print(f"[TX] Transmit Now triggered (Session #{my_session})! Enqueuing snapshot...")

    # Compose snapshot
    bx, by, bw, bh = target_x, target_y, target_w, target_h
    current_sensor = base_sensor_image.copy()
    draw = ImageDraw.Draw(current_sensor)
    # Render active target into sensor frame
    draw_person(draw, int(bx), int(by))

    snap = build_tx_snapshot_opposite_pip(current_sensor, bx, by, bw, bh, target_conf)

    t = threading.Thread(target=broadcast_worker, args=(snap, target_conf, 2, my_session, (bx, by, bw, bh)), daemon=True)
    t.start()

def stop_broadcast_now():
    global tx_active, tx_session_id
    tx_session_id += 1
    tx_active = False
    if btn_transmit:
        btn_transmit.config(text="Transmit Now", bg=C_BLUE, fg="#FFFFFF")
    if lbl_tx_progress:
        lbl_tx_progress.config(text="TRANSMITTER STANDBY (CLICK TO BROADCAST)", fg=C_SUB)

def clear_phone_screen_now():
    globals()["last_click_time"] = time.time()
    stop_broadcast_now()
    if lbl_tx_progress:
        lbl_tx_progress.config(text="CLEARING PHONE SCREEN (CMD:CLEAR)...", fg=C_AMBER)
    def _clr_worker():
        time.sleep(0.04)
        for _ in range(4):
            virtual_lora_send("CMD:CLEAR")
            time.sleep(0.06)
        if lbl_tx_progress:
            lbl_tx_progress.config(text="PHONE SCREEN CLEARED (STANDBY)", fg=C_SUB)
    threading.Thread(target=_clr_worker, daemon=True).start()
    print("[TX] CMD:CLEAR sent to ground receiver")

def send_config_async():
    threading.Thread(target=broadcast_config_to_receiver, daemon=True).start()

# --- Reverse UDP Listener (Commands from Phone Dashboard) ---
def reverse_cmd_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", DEFAULT_SENDER_LISTEN_PORT))
    except Exception as e:
        print(f"[Reverse Listener Warning] Bind error on port {DEFAULT_SENDER_LISTEN_PORT}: {e}")
        return
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            cmd = data.decode("utf-8").strip()
            if cmd:
                ui_cmd_queue.put(cmd)   # executed on the Tk main thread
        except Exception:
            pass

def process_ui_commands():
    """Runs on the Tk main thread: applies commands posted by background threads."""
    while True:
        try:
            cmd = ui_cmd_queue.get_nowait()
        except queue.Empty:
            return
        try:
            if cmd == "CMD:CLEAR":
                stop_broadcast_now()
            elif cmd.startswith("CMD:LEVEL:"):
                select_lora_level_btn(int(cmd.split(":")[2]))
            elif cmd.startswith("CMD:RES:"):
                parts = cmd.split(":")
                w, h = int(parts[2]), int(parts[3])
                for r_code, r_lbl, rw, rh, _, _, _ in RES_PRESETS:
                    if rw == w and rh == h and (rw != TX_W or rh != TX_H):
                        select_tx_res_btn(r_code, rw, rh)
                        break
        except Exception as e:
            print(f"[UI Command Error] {cmd}: {e}")

# --- Mission Control GUI (Tkinter + Auterion/DJI Dark Aesthetic) ---
BG_DARK   = "#111317"
BG_PANEL  = "#1A1D24"
BG_INPUT  = "#12151B"
BD_LINE   = "#2A2F3A"
C_BLUE    = "#2563EB"
C_BLUE_HI = "#3B82F6"
C_GREEN   = "#10B981"
C_AMBER   = "#F59E0B"
C_RED     = "#EF4444"
C_TEXT    = "#F3F4F6"
C_SUB     = "#9CA3AF"

F_TITLE   = ("Segoe UI", 12, "bold")
F_HEAD    = ("Segoe UI", 10, "bold")
F_SANS    = ("Segoe UI", 9)
F_MONO    = ("Consolas", 9)

root = None
canvas_wide = None
canvas_scope = None
tk_wide_img = None
tk_scope_img = None
tk_wide_fb = None
tk_scope_fb = None

def pil_to_tk_canvas(pil_img, tk_target):
    w, h = pil_img.size
    ppm = f"P6 {w} {h} 255\n".encode("ascii") + pil_img.tobytes()
    tmp = tk.PhotoImage(data=ppm)
    tk_target.tk.call(tk_target, "copy", tmp, "-to", 0, 0)

mode_buttons = {}
res_buttons = {}
level_buttons = {}
model_buttons = {}

lbl_lock_badge = None
lbl_telemetry_box = None
lbl_tx_progress = None
btn_transmit = None
sim_loss_slider = None
sim_rssi_slider = None

def select_tx_mode_btn(m_code):
    global active_tx_mode
    active_tx_mode = m_code
    for k, btn in mode_buttons.items():
        btn.config(bg=C_BLUE if k == m_code else BG_INPUT, fg="#FFFFFF" if k == m_code else C_TEXT)
    send_config_async()

def select_tx_res_btn(r_code, w, h):
    global active_res_code, TX_W, TX_H
    active_res_code = r_code
    TX_W = w
    TX_H = h
    for k, btn in res_buttons.items():
        btn.config(bg=C_BLUE if k == r_code else BG_INPUT, fg="#FFFFFF" if k == r_code else C_TEXT)
    if tx_active:
        stop_broadcast_now()   # old-resolution blocks must never land on the new grid
    send_config_async()

def select_lora_level_btn(lvl):
    global current_level
    current_level = lvl
    for k, btn in level_buttons.items():
        btn.config(bg=C_BLUE if k == lvl else BG_INPUT, fg="#FFFFFF" if k == lvl else C_TEXT)
    def _lvl_worker():
        for _ in range(3):
            virtual_lora_send(f"CMD:LEVEL:{lvl}")
            time.sleep(0.05)
        broadcast_config_to_receiver()
    threading.Thread(target=_lvl_worker, daemon=True).start()

def select_ai_model_btn(m_name):
    global active_model_name
    active_model_name = m_name
    for k, btn in model_buttons.items():
        btn.config(bg=C_BLUE if k == m_name else BG_INPUT, fg="#FFFFFF" if k == m_name else C_TEXT)
    send_config_async()

def on_wide_canvas_click(event):
    global target_x, target_y, cand_lock_count, confirm_lock_count, zoom_state
    # Map click coordinates from the 4:3 wide canvas to the 2028x1520 sensor frame
    cw = float(WIDE_DISP_W)
    ch = float(WIDE_DISP_H)
    target_x = max(20.0, min(BINNED_W - 60.0, (event.x / cw) * BINNED_W))
    target_y = max(20.0, min(BINNED_H - 80.0, (event.y / ch) * BINNED_H))
    cand_lock_count = 0
    confirm_lock_count = 0
    zoom_state = "WIDE_SEARCH"
    globals()["last_auto_tx_time"] = 0.0   # moving the target re-arms auto-transmit

def load_custom_image():
    global base_sensor_image
    fpath = filedialog.askopenfilename(filetypes=[("Image Files", "*.jpg;*.jpeg;*.png;*.bmp")])
    if fpath:
        try:
            im = Image.open(fpath).convert("RGB")
            base_sensor_image = im.resize((BINNED_W, BINNED_H), Image.Resampling.BILINEAR)
            refresh_wide_base()
        except Exception as e:
            print(f"[Image Load Error] {e}")

def reset_to_synthetic():
    init_base_terrain_image()

# --- Simulation Update Loop (Runs at ~30 FPS) ---
auto_tx_var = None
last_click_time = 0.0

def update_simulation_loop():
    global target_x, target_y, target_drift_vx, target_drift_vy, target_conf
    global gimbal_cx, gimbal_cy, zoom_state, cand_lock_count, confirm_lock_count
    global last_auto_tx_time, heartbeat_seq, tk_wide_img, tk_scope_img
    global sim_packet_loss_pct, sim_rssi_dbm, sim_snr_db, last_hb_time

    try:
        process_ui_commands()

        # Read slider values
        if sim_loss_slider:
            sim_packet_loss_pct = float(sim_loss_slider.get())
        if sim_rssi_slider:
            sim_rssi_dbm = float(sim_rssi_slider.get())
            sim_snr_db = max(-15.0, min(12.0, (sim_rssi_dbm + 95.0) / 4.0))

        # 1. Subtle target walking drift
        target_x += target_drift_vx
        target_y += target_drift_vy
        if target_x < 150 or target_x > BINNED_W - 150: target_drift_vx *= -1
        if target_y < 150 or target_y > BINNED_H - 150: target_drift_vy *= -1

        # Damped gimbal tracking scope
        gimbal_cx += (target_x + target_w * 0.5 - gimbal_cx) * 0.15
        gimbal_cy += (target_y + target_h * 0.5 - gimbal_cy) * 0.15

        # Simulated IMX500 detection confidence (higher once the scope has zoomed in)
        base_c = 72.0 if zoom_state == "WIDE_SEARCH" else 84.0
        target_conf = max(40.0, min(97.0, target_conf + (base_c - target_conf) * 0.05 + random.uniform(-2.0, 2.0)))

        # 2. 2-Stage Lock State Machine (WIDE_SEARCH -> ZOOM_CONFIRM -> LOCKED)
        now = time.time()
        wide_thresh = 38.0
        confirm_thresh = 58.0
        if zoom_state == "WIDE_SEARCH":
            if target_conf >= wide_thresh:
                cand_lock_count = min(3, cand_lock_count + 1)
                if cand_lock_count >= 3:
                    zoom_state = "ZOOM_CONFIRM"
                    confirm_lock_count = 0
            else:
                cand_lock_count = 0
        elif zoom_state == "ZOOM_CONFIRM":
            if target_conf >= confirm_thresh:
                confirm_lock_count = min(3, confirm_lock_count + 1)
                if confirm_lock_count >= 3:
                    zoom_state = "LOCKED"
                    auto_on = bool(auto_tx_var.get()) if auto_tx_var is not None else True
                    if auto_on and not tx_active and (now - last_auto_tx_time) > AUTO_TX_COOLDOWN_S:
                        last_auto_tx_time = now
                        print("[AUTO-TX] Target confirmed -> broadcasting snapshot")
                        trigger_broadcast_now()
            else:
                confirm_lock_count = 0
        # LOCKED stays locked (no re-broadcast spam) until the target is moved by a click.

        # 3. Wide optical viewport (pre-scaled 4:3 base, so this is cheap every frame)
        sx = WIDE_DISP_W / float(BINNED_W)
        sy = WIDE_DISP_H / float(BINNED_H)
        wide_img = wide_base_small.copy()
        dw = ImageDraw.Draw(wide_img)
        hx, hy = int(target_x * sx), int(target_y * sy)
        dw.ellipse([hx - 1, hy - 1, hx + 5, hy + 5], fill=(255, 110, 30))
        box_col = (16, 185, 129) if zoom_state == "LOCKED" else ((245, 158, 11) if zoom_state == "ZOOM_CONFIRM" else (239, 68, 68))
        bx0, by0 = int((target_x - 6) * sx), int((target_y - 10) * sy)
        bx1, by1 = int((target_x + target_w) * sx), int((target_y + target_h) * sy)
        dw.rectangle([bx0 - 3, by0 - 3, bx1 + 3, by1 + 3], outline=box_col, width=2)
        dw.text((bx0 - 3, max(0, by0 - 16)), f"PERSON {target_conf:.0f}%", fill=box_col)

        scope_w = int(BINNED_W / gimbal_zoom)
        scope_h = int(scope_w * SCOPE_DISP_H / float(SCOPE_DISP_W))
        crop_x0 = int(max(0, min(BINNED_W - scope_w, gimbal_cx - scope_w * 0.5)))
        crop_y0 = int(max(0, min(BINNED_H - scope_h, gimbal_cy - scope_h * 0.5)))
        dw.rectangle([int(crop_x0 * sx), int(crop_y0 * sy), int((crop_x0 + scope_w) * sx), int((crop_y0 + scope_h) * sy)],
                     outline=(59, 130, 246), width=2)
        dw.text((int(crop_x0 * sx) + 3, int(crop_y0 * sy) + 2), f"SCOPE {gimbal_zoom:.1f}X", fill=(147, 197, 253))
        pil_to_tk_canvas(wide_img, tk_wide_fb)

        # 4. Zoomed tracking scope: crop the full-res sensor and draw the person inside it
        scope_crop = base_sensor_image.crop((crop_x0, crop_y0, crop_x0 + scope_w, crop_y0 + scope_h))
        draw_person(ImageDraw.Draw(scope_crop), int(target_x) - crop_x0, int(target_y) - crop_y0)
        scope_img = scope_crop.resize((SCOPE_DISP_W, SCOPE_DISP_H), Image.Resampling.BILINEAR)
        ds = ImageDraw.Draw(scope_img)
        k = SCOPE_DISP_W / float(scope_w)
        px0 = int((target_x - 8 - crop_x0) * k); py0 = int((target_y - 12 - crop_y0) * k)
        px1 = int((target_x + target_w - crop_x0) * k); py1 = int((target_y + target_h - crop_y0) * k)
        ds.rectangle([px0, py0, px1, py1], outline=box_col, width=2)
        ds.text((px0, max(0, py0 - 14)), f"TGT {target_conf:.0f}%", fill=box_col)
        sc_cx, sc_cy = SCOPE_DISP_W // 2, SCOPE_DISP_H // 2
        ds.line([(sc_cx - 18, sc_cy), (sc_cx + 18, sc_cy)], fill=(59, 130, 246), width=1)
        ds.line([(sc_cx, sc_cy - 18), (sc_cx, sc_cy + 18)], fill=(59, 130, 246), width=1)
        pil_to_tk_canvas(scope_img, tk_scope_fb)

        # 5. Status badges, Transmit button and progress (all driven from state, on the Tk thread)
        if lbl_lock_badge:
            if zoom_state == "WIDE_SEARCH":
                lbl_lock_badge.config(text=f"WIDE_SEARCH · LOCK {cand_lock_count}/3", bg=BG_INPUT, fg=C_AMBER)
            elif zoom_state == "ZOOM_CONFIRM":
                lbl_lock_badge.config(text=f"ZOOM_CONFIRM · LOCK {confirm_lock_count}/3", bg=C_BLUE, fg="#FFFFFF")
            else:
                auto_on = bool(auto_tx_var.get()) if auto_tx_var is not None else False
                if auto_on:
                    lbl_lock_badge.config(text="TARGET LOCKED · AUTO TX", bg=C_GREEN, fg="#FFFFFF")
                else:
                    lbl_lock_badge.config(text="TARGET LOCKED (STANDBY)", bg=C_GREEN, fg="#FFFFFF")

        if btn_transmit:
            want = ("● BROADCASTING...", C_GREEN) if tx_active else ("Transmit Now", C_BLUE)
            if btn_transmit.cget("text") != want[0]:
                btn_transmit.config(text=want[0], bg=want[1], fg="#FFFFFF")

        if lbl_tx_progress:
            if tx_active:
                lvl = LEVEL_CONFIGS[current_level]
                if tx_progress_total == 0 or tx_progress_idx == 0:
                    lbl_tx_progress.config(text=f"TX PASS {tx_pass}: SENDING D:/M:/JH: HEADERS ({lvl[5].split(' (')[0]})", fg=C_AMBER)
                else:
                    pct = int((tx_progress_idx / max(1, tx_progress_total)) * 100)
                    lbl_tx_progress.config(
                        text=f"TX PASS {tx_pass}/2: {tx_progress_idx} / {tx_progress_total} ({pct}%) · {lvl[5].split(' (')[0]}",
                        fg=C_BLUE_HI)
            elif not lbl_tx_progress.cget("text").startswith("CLEARING") or (now - last_click_time) > 1.5:
                lbl_tx_progress.config(text="TRANSMITTER STANDBY", fg=C_SUB)

        # 6. Heartbeat beacon exactly every 2.5 s while idle (keeps the phone link indicator green)
        if not tx_active and (now - last_hb_time) >= HB_INTERVAL_S:
            last_hb_time = now
            heartbeat_seq += 1
            hb_pkt = (f"H:{heartbeat_seq}:{zoom_state}:{target_conf:.1f}:{active_tx_mode}:{active_model_name}:"
                      f"{TX_W}:{TX_H}:{gps_latitude};{gps_longitude};{gps_altitude}")
            virtual_lora_send(hb_pkt)
    except Exception as e:
        print(f"[Sim Loop Error] {e}")

    root.after(33, update_simulation_loop)

# --- GUI Construction ---
def create_gui():
    global root, canvas_wide, canvas_scope, lbl_lock_badge, lbl_tx_progress
    global sim_loss_slider, sim_rssi_slider

    root = tk.Tk()
    root.title("SAR Drone AI Mission Control — Virtual Hardware Simulator")
    root.geometry("1120x790")
    root.configure(bg=BG_DARK)

    # Top Mission Control Bar
    top_bar = tk.Frame(root, bg=BG_PANEL, height=42, padx=12, pady=6)
    top_bar.pack(fill=tk.X, side=tk.TOP)
    tk.Label(top_bar, text="SAR AIRBORNE MISSION CONTROL · 868 MHz LoRa", font=F_TITLE, bg=BG_PANEL, fg=C_TEXT).pack(side=tk.LEFT)
    lbl_tx_progress = tk.Label(top_bar, text="TRANSMITTER STANDBY", font=F_HEAD, bg=BG_PANEL, fg=C_SUB)
    lbl_tx_progress.pack(side=tk.RIGHT, padx=10)

    # Main Split Container
    main_frame = tk.Frame(root, bg=BG_DARK)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

    # Left Column: Stacked Dual Optical Viewports (460px width)
    left_col = tk.Frame(main_frame, bg=BG_DARK, width=470)
    left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

    # Top Viewport Header
    w_hdr = tk.Frame(left_col, bg=BG_PANEL, padx=8, pady=4)
    w_hdr.pack(fill=tk.X)
    tk.Label(w_hdr, text="WIDE OPTICAL SENSOR (2028×1520)", font=F_HEAD, bg=BG_PANEL, fg=C_TEXT).pack(side=tk.LEFT)
    tk.Label(w_hdr, text="CLICK TO MOVE TARGET", font=F_SANS, bg=BG_PANEL, fg=C_AMBER).pack(side=tk.RIGHT)

    canvas_wide = tk.Canvas(left_col, width=WIDE_DISP_W, height=WIDE_DISP_H, bg="#090B0E", highlightthickness=1, highlightbackground=BD_LINE)
    canvas_wide.pack(pady=(2, 8))
    canvas_wide.bind("<Button-1>", on_wide_canvas_click)
    global tk_wide_fb, tk_scope_fb
    tk_wide_fb = tk.PhotoImage(width=WIDE_DISP_W, height=WIDE_DISP_H)
    canvas_wide.create_image(0, 0, anchor=tk.NW, image=tk_wide_fb)

    # Bottom Viewport Header
    s_hdr = tk.Frame(left_col, bg=BG_PANEL, padx=8, pady=4)
    s_hdr.pack(fill=tk.X)
    tk.Label(s_hdr, text="TARGET TRACKING SCOPE (GIMBAL ZOOM)", font=F_HEAD, bg=BG_PANEL, fg=C_TEXT).pack(side=tk.LEFT)
    lbl_lock_badge = tk.Label(s_hdr, text="WIDE_SEARCH", font=F_SANS, bg=BG_INPUT, fg=C_AMBER, padx=6, pady=1)
    lbl_lock_badge.pack(side=tk.RIGHT)

    canvas_scope = tk.Canvas(left_col, width=SCOPE_DISP_W, height=SCOPE_DISP_H, bg="#090B0E", highlightthickness=1, highlightbackground=BD_LINE)
    canvas_scope.pack(pady=(2, 4))
    tk_scope_fb = tk.PhotoImage(width=SCOPE_DISP_W, height=SCOPE_DISP_H)
    canvas_scope.create_image(0, 0, anchor=tk.NW, image=tk_scope_fb)

    # Right Column: Controls, Sliders & LoRa Settings
    right_col = tk.Frame(main_frame, bg=BG_PANEL, padx=12, pady=10)
    right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # 1. Action Buttons Row
    global btn_transmit
    btn_row = tk.Frame(right_col, bg=BG_PANEL)
    btn_row.pack(fill=tk.X, pady=(0, 10))
    btn_transmit = tk.Button(btn_row, text="Transmit Now", bg=C_BLUE, fg="#FFFFFF", font=F_HEAD, padx=12, pady=6, bd=0, cursor="hand2", command=trigger_broadcast_now)
    btn_transmit.pack(side=tk.LEFT, padx=(0, 6))
    tk.Button(btn_row, text="Stop Broadcast", bg=BG_INPUT, fg=C_RED, font=F_HEAD, padx=10, pady=6, bd=1, cursor="hand2", command=stop_broadcast_now).pack(side=tk.LEFT, padx=(0, 6))
    tk.Button(btn_row, text="Clear Phone Screen", bg=BG_INPUT, fg=C_TEXT, font=F_SANS, padx=10, pady=6, bd=1, cursor="hand2", command=clear_phone_screen_now).pack(side=tk.LEFT, padx=(0, 6))
    tk.Button(btn_row, text="Load Image...", bg=BG_INPUT, fg=C_SUB, font=F_SANS, padx=8, pady=6, bd=1, cursor="hand2", command=load_custom_image).pack(side=tk.RIGHT)
    global auto_tx_var
    auto_tx_var = tk.IntVar(value=0)
    tk.Checkbutton(right_col, text="Auto-transmit when a target is confirmed (2-stage lock)", variable=auto_tx_var,
                   bg=BG_PANEL, fg=C_TEXT, selectcolor=BG_INPUT, activebackground=BG_PANEL, activeforeground=C_TEXT,
                   font=F_SANS).pack(anchor=tk.W, pady=(0, 6))
    tk.Button(btn_row, text="Reset Scene", bg=BG_INPUT, fg=C_SUB, font=F_SANS, padx=8, pady=6, bd=1, cursor="hand2", command=reset_to_synthetic).pack(side=tk.RIGHT, padx=4)

    # 2. Picture Resolution Selection
    tk.Label(right_col, text="TRANSMISSION RESOLUTION (GRID SIZE)", font=F_HEAD, bg=BG_PANEL, fg=C_SUB).pack(anchor=tk.W, pady=(4, 2))
    res_row = tk.Frame(right_col, bg=BG_PANEL)
    res_row.pack(fill=tk.X, pady=(0, 8))
    for r_code, r_lbl, rw, rh, _, _, _ in RES_PRESETS:
        btn = tk.Button(res_row, text=r_lbl, bg=C_BLUE if r_code == active_res_code else BG_INPUT,
                        fg="#FFFFFF" if r_code == active_res_code else C_TEXT, font=F_SANS, padx=6, pady=4, bd=0, cursor="hand2",
                        command=lambda c=r_code, w=rw, h=rh: select_tx_res_btn(c, w, h))
        btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        res_buttons[r_code] = btn

    # 3. LoRa Preset Speed Selection (L0 - L7)
    tk.Label(right_col, text="LORA SPEED PRESET (L0 TO L7)", font=F_HEAD, bg=BG_PANEL, fg=C_SUB).pack(anchor=tk.W, pady=(4, 2))
    lora_row = tk.Frame(right_col, bg=BG_PANEL)
    lora_row.pack(fill=tk.X, pady=(0, 8))
    for lvl, sf, bw, cr, gap, name in LEVEL_CONFIGS:
        btn = tk.Button(lora_row, text=f"L{lvl}", bg=C_BLUE if lvl == current_level else BG_INPUT,
                        fg="#FFFFFF" if lvl == current_level else C_TEXT, font=F_SANS, padx=4, pady=4, bd=0, cursor="hand2",
                        command=lambda l=lvl: select_lora_level_btn(l))
        btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        level_buttons[lvl] = btn

    # 4. Picture Transmission Mode
    tk.Label(right_col, text="PICTURE FORMAT MODE", font=F_HEAD, bg=BG_PANEL, fg=C_SUB).pack(anchor=tk.W, pady=(4, 2))
    mode_row = tk.Frame(right_col, bg=BG_PANEL)
    mode_row.pack(fill=tk.X, pady=(0, 8))
    for m_code, m_lbl, m_desc in TX_MODES:
        btn = tk.Button(mode_row, text=m_lbl, bg=C_BLUE if m_code == active_tx_mode else BG_INPUT,
                        fg="#FFFFFF" if m_code == active_tx_mode else C_TEXT, font=F_SANS, padx=6, pady=4, bd=0, cursor="hand2",
                        command=lambda m=m_code: select_tx_mode_btn(m))
        btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        mode_buttons[m_code] = btn

    # 5. AI Detector Selection
    tk.Label(right_col, text="AI DETECTOR MODEL", font=F_HEAD, bg=BG_PANEL, fg=C_SUB).pack(anchor=tk.W, pady=(4, 2))
    ai_row = tk.Frame(right_col, bg=BG_PANEL)
    ai_row.pack(fill=tk.X, pady=(0, 10))
    for m_name, m_full in AI_MODELS:
        btn = tk.Button(ai_row, text=m_name, bg=C_BLUE if m_name == active_model_name else BG_INPUT,
                        fg="#FFFFFF" if m_name == active_model_name else C_TEXT, font=F_SANS, padx=6, pady=4, bd=0, cursor="hand2",
                        command=lambda mn=m_name: select_ai_model_btn(mn))
        btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        model_buttons[m_name] = btn

    # 6. Virtual RF Simulation Controls (Packet Loss & RSSI sliders)
    rf_box = tk.LabelFrame(right_col, text=" Virtual RF Channel Simulation Controls ", font=F_HEAD, bg=BG_PANEL, fg=C_AMBER, padx=8, pady=6)
    rf_box.pack(fill=tk.X, pady=(0, 8))

    # Loss Slider
    loss_f = tk.Frame(rf_box, bg=BG_PANEL)
    loss_f.pack(fill=tk.X, pady=2)
    tk.Label(loss_f, text="Packet Loss Rate (%):", font=F_SANS, bg=BG_PANEL, fg=C_TEXT, width=18, anchor=tk.W).pack(side=tk.LEFT)
    sim_loss_slider = tk.Scale(loss_f, from_=0, to=30, orient=tk.HORIZONTAL, bg=BG_PANEL, fg=C_TEXT, highlightthickness=0)
    sim_loss_slider.set(0)
    sim_loss_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

    # RSSI Slider
    rssi_f = tk.Frame(rf_box, bg=BG_PANEL)
    rssi_f.pack(fill=tk.X, pady=2)
    tk.Label(rssi_f, text="Signal Power (dBm):", font=F_SANS, bg=BG_PANEL, fg=C_TEXT, width=18, anchor=tk.W).pack(side=tk.LEFT)
    sim_rssi_slider = tk.Scale(rssi_f, from_=-45, to=-125, orient=tk.HORIZONTAL, bg=BG_PANEL, fg=C_TEXT, highlightthickness=0)
    sim_rssi_slider.set(-68)
    sim_rssi_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

    # Telemetry Info Card
    info_card = tk.Frame(right_col, bg=BG_INPUT, padx=8, pady=8, highlightthickness=1, highlightbackground=BD_LINE)
    info_card.pack(fill=tk.BOTH, expand=True)
    tk.Label(info_card, text="SYSTEM STATUS & RF DIAGNOSTICS", font=F_HEAD, bg=BG_INPUT, fg=C_SUB).pack(anchor=tk.W, pady=(0, 4))
    diag_txt = (
        f"• Local LoRa UDP Target:  127.0.0.1:{DEFAULT_LORA_UDP_PORT}\n"
        f"• Sony IMX500 Resolution: 2028×1520 @ 30 FPS (Full Sensor)\n"
        f"• Auto-Opposite PiP Inset: Enabled (Zooms Target in Diagonally Opposite Corner)\n"
        f"• Target-First Radial Sort: Enabled (Candidate Person Blocks Stream First)\n"
        f"• Pass 2 Rescan Repair:    Active (Fills any missed packet drops)\n"
        f"• Phone Web Server:        Open your browser to view the ground HUD!"
    )
    tk.Label(info_card, text=diag_txt, font=F_MONO, bg=BG_INPUT, fg=C_TEXT, justify=tk.LEFT).pack(anchor=tk.W)

if __name__ == "__main__":
    init_base_terrain_image()
    # Start reverse command listener
    t_rev = threading.Thread(target=reverse_cmd_listener, daemon=True)
    t_rev.start()

    create_gui()
    broadcast_config_to_receiver()
    update_simulation_loop()
    root.mainloop()
