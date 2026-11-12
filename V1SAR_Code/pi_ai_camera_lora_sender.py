import os
import sys
import io
import glob
import time
import math
import base64
import datetime
import threading
import tkinter as tk
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

# 1. Load Waveshare's SX126x SPI driver
sys.path.insert(0, "/home/rhys/sx126x_lorawan_hat_code/python/lora")
from LoRaRF import SX126x

# 2. Load Official Picamera2 & Sony IMX500 AI Camera Object Detection classes
HAS_AI_CAM = False
try:
    from picamera2 import Picamera2
    from picamera2.devices.imx500 import IMX500, NetworkIntrinsics, postprocess_nanodet_detection
    from picamera2.devices.imx500.postprocess import scale_boxes
    from libcamera import Transform
    HAS_AI_CAM = True
except Exception as e:
    print(f"[Warning] Picamera2 / IMX500 import warning: {e}")

# Native 2x2 Binned Sensor Mode from Raspberry Pi AI Camera Product Brief: 2028 x 1520 @ 30fps
BINNED_W = 2028
BINNED_H = 1520

# Stacked Vertical Video Windows (One on top of the other on the Left Column)
WIDE_DISP_W = 460
WIDE_DISP_H = 285
ZOOM_DISP_W = 460
ZOOM_DISP_H = 245

# Broadcast JPEG Output Resolution (Selectable from 320x240 up to 1280x960 from the 2028x1520 sensor!)
TX_W = 640
TX_H = 480
TILE_W = 64
TILE_H = 16
TILE_COLS = TX_W // TILE_W   # 10
TILE_ROWS = TX_H // TILE_H   # 30

# Selectable Broadcast Resolutions (Code, Button Label, Width, Height)
TX_RESOLUTIONS = [
    ("320x240",  "320×240 (75)",   320,  240),
    ("640x480",  "640×480 (300)",  640,  480),
    ("960x720",  "960×720 (675)",  960,  720),
    ("1280x960", "1280×960 (1200)", 1280, 960),
]
active_res_code = "640x480"

# Local Broadcast Log Directory on Raspberry Pi
PI_LOG_DIR = os.path.expanduser("~/Documents/SearchRescueDroneProject/broadcast_logs")
os.makedirs(PI_LOG_DIR, exist_ok=True)

B64_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

# 8 Speed Levels matching ESP32-S3 SX1262 V4 (sf, bw, cr, ldro, safe_pkt_gap_ms, short_lbl, full_lbl)
LEVEL_CONFIGS = {
    0: (12, 125000, 5, True,  1450, "L0", "LEVEL 0 (SF12/125k Extreme)"),
    1: (11, 125000, 5, True,  780,  "L1", "LEVEL 1 (SF11/125k V.Long)"),
    2: (10, 125000, 5, False, 410,  "L2", "LEVEL 2 (SF10/125k Long)"),
    3: (9,  250000, 5, False, 175,  "L3", "LEVEL 3 (SF9/250k Medium)"),
    4: (8,  250000, 5, False, 110,  "L4", "LEVEL 4 (SF8/250k Med-Fast)"),
    5: (7,  250000, 5, False, 75,   "L5", "LEVEL 5 (SF7/250k Fast)"),
    6: (7,  500000, 5, False, 55,   "L6", "LEVEL 6 (SF7/500k Turbo)"),
    7: (6,  500000, 5, False, 45,   "L7", "LEVEL 7 (SF6/500k Max)"),
}

# Broadcast Modes (Code, Button Label, Description)
TX_MODES = [
    ("SCAN_TARGET", "Target-First Blocks", "64x16 Picture Blocks (Person First -> Outward)"),
    ("SCAN_ROWS",   "Row-Order Blocks",    "64x16 Picture Blocks (Top-to-Bottom Grid)"),
    ("2D_FULL",     "Full-Frame Packets",  "Full-Frame Progressive JPEG Packets"),
]

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

# --- Initialize Waveshare SX1262 LoRa Node SPI HAT as TRANSMITTER ---
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
print("Initializing Waveshare SX1262 SPI HAT as AI Drone Transmitter (CS=GPIO21, RST=18, BUSY=20, TXEN=6)...")
if not LoRa.begin(0, 0, 18, 20, -1, 6, -1):
    raise RuntimeError("Could not initialize Waveshare SX1262 SPI HAT! Check SPI and dtoverlay=spi0-0cs,cs0_pin=21")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

current_level = 5
active_tx_mode = "SCAN_TARGET"
last_lora_tx_time = 0.0
heartbeat_seq = 0

def set_lora_level(level_num):
    global current_level
    if level_num not in LEVEL_CONFIGS:
        return
    sf, bw, cr, ldro, _, _, name = LEVEL_CONFIGS[level_num]
    with lora_lock:
        current_level = level_num
        LoRa.setStandby(LoRa.STANDBY_RC)
        LoRa.setLoRaModulation(sf, bw, cr, ldro)
        LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 8, 255, True, False)
        LoRa.setSyncWord(0x1424)
        print(f"Waveshare SX1262 SPI Transmitter ready on {name} @ 868.0 MHz (+22 dBm)!")

set_lora_level(current_level)

def lora_send_line(line_str):
    global last_lora_tx_time
    clean = line_str.strip()
    raw = [ord(c) for c in clean if 32 <= ord(c) < 127]
    if not raw or len(raw) > 250:
        return
    with lora_lock:
        try:
            LoRa.beginPacket()
            LoRa.write(raw, len(raw))
            LoRa.endPacket()
            LoRa.wait()
            last_lora_tx_time = time.time()
        except Exception as e:
            print(f"[LoRa TX Warning] {e}")

# --- Optional Background NMEA GPS Reader ---
auto_gps_fix = None

def parse_nmea_gga_rmc(line):
    global auto_gps_fix
    try:
        parts = line.strip().split(",")
        if line.startswith(("$GPGGA", "$GNGGA")) and len(parts) > 9 and parts[2] and parts[4]:
            lat_raw, ns, lon_raw, ew, fix_q, sats, _, alt = parts[2], parts[3], parts[4], parts[5], parts[6], parts[7], parts[8], parts[9]
            if fix_q != "0":
                lat_deg = int(float(lat_raw) / 100) + (float(lat_raw) % 100) / 60.0
                lon_deg = int(float(lon_raw) / 100) + (float(lon_raw) % 100) / 60.0
                auto_gps_fix = f"{lat_deg:.5f}{ns}, {lon_deg:.5f}{ew} ({alt}m, {sats}sat)"
    except Exception:
        pass

def gps_reader_worker():
    try:
        import serial
    except ImportError:
        return
    while True:
        for dev in ("/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyAMA0", "/dev/serial0"):
            if os.path.exists(dev):
                try:
                    with serial.Serial(dev, 9600, timeout=1.5) as ser:
                        while True:
                            ln = ser.readline().decode("ascii", errors="ignore")
                            if ln.startswith("$G"):
                                parse_nmea_gga_rmc(ln)
                except Exception:
                    pass
        time.sleep(5.0)

threading.Thread(target=gps_reader_worker, daemon=True).start()

# --- Official Raspberry Pi AI Camera Bounding-Box Models ---
MODEL_CANDIDATES = [
    ("SSD MobileNet", "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"),
    ("EfficientDet",  "/usr/share/imx500-models/imx500_network_efficientdet_lite0_pp.rpk"),
    ("YOLOv8n",       "/usr/share/imx500-models/imx500_network_yolov8n_pp.rpk"),
    ("NanoDet",       "/usr/share/imx500-models/imx500_network_nanodet_plus_416x416_pp.rpk"),
]

available_models = [(name, path) for (name, path) in MODEL_CANDIDATES if os.path.exists(path)]
if os.path.isdir("/usr/share/imx500-models") and not available_models:
    for p in sorted(glob.glob("/usr/share/imx500-models/*.rpk")):
        bname = os.path.basename(p).lower()
        if any(k in bname for k in ("ssd", "eff", "yolo", "nano", "det")) and "hrnet" not in bname and "posenet" not in bname:
            available_models.append((os.path.basename(p)[:14], p))

imx500 = None
picam2 = None
intrinsics = None
active_model_name = available_models[0][0] if available_models else "None"
cam_lock = threading.Lock()
last_valid_detections = []

def load_ai_model(model_name, model_path, do_vflip=True, do_hflip=True):
    """Configures the IMX500 at its native 2x2 Binned 2028x1520 30fps mode."""
    global imx500, picam2, intrinsics, active_model_name, last_valid_detections
    if not HAS_AI_CAM:
        return
    with cam_lock:
        try:
            last_valid_detections = []
            if picam2 is not None:
                try:
                    picam2.stop()
                    picam2.close()
                except Exception:
                    pass
                picam2 = None
                time.sleep(0.25)

            print(f"Loading Sony IMX500 Box Model (2x2 Binned 2028x1520 @ 30fps): {model_name}...")
            imx500 = IMX500(model_path)
            intrinsics = imx500.network_intrinsics
            if not intrinsics:
                intrinsics = NetworkIntrinsics()
                intrinsics.task = "object detection"
            if "yolov8" in model_path.lower():
                intrinsics.bbox_normalization = True
                intrinsics.bbox_order = "xy"
            intrinsics.update_with_defaults()

            picam2 = Picamera2(imx500.camera_num)
            cam_cfg = picam2.create_preview_configuration(
                main={"size": (BINNED_W, BINNED_H), "format": "RGB888"},
                raw={"size": (BINNED_W, BINNED_H)},
                transform=Transform(hflip=1 if do_hflip else 0, vflip=1 if do_vflip else 0),
                controls={"FrameRate": 30},
                buffer_count=6
            )
            imx500.show_network_fw_progress_bar()
            picam2.start(cam_cfg, show_preview=False)
            active_model_name = model_name
            print(f"IMX500 2x2 Binned (2028x1520 @ 30fps) running [{model_name}]!")
        except Exception as e:
            print(f"[AI Camera Load Error] {e}")
            imx500 = None
            picam2 = None

if HAS_AI_CAM and available_models:
    m_name, m_path = available_models[0]
    load_ai_model(m_name, m_path, do_vflip=True, do_hflip=True)

def broadcast_config_to_receiver():
    """Immediately broadcasts active Level, Image Mode, AI Model, and Resolution to the ESP32-S3 Receiver!"""
    def _worker():
        cfg_pkt = f"CFG:{current_level}:{active_tx_mode}:{active_model_name}:{thresh_var.get()}:{TX_W}:{TX_H}"
        for _ in range(3):
            lora_send_line(cfg_pkt)
            time.sleep(0.09)
    threading.Thread(target=_worker, daemon=True).start()

def parse_detections_2028(metadata, min_conf_pct=35.0, person_only=True, single_best_only=True):
    global last_valid_detections
    if imx500 is None or picam2 is None or metadata is None:
        return last_valid_detections
    try:
        np_outputs = imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            return last_valid_detections

        input_w, input_h = imx500.get_input_size()
        bbox_normalization = getattr(intrinsics, "bbox_normalization", False)
        bbox_order = getattr(intrinsics, "bbox_order", "yx")

        if getattr(intrinsics, "postprocess", "") == "nanodet":
            boxes, scores, classes = postprocess_nanodet_detection(
                outputs=np_outputs[0], conf=min_conf_pct / 100.0, iou_thres=0.65, max_out_dets=5
            )[0]
            boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
        else:
            boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
            if bbox_normalization:
                boxes = boxes / float(input_h)
            if bbox_order == "xy":
                boxes = boxes[:, [1, 0, 3, 2]]
            boxes = np.array_split(boxes, 4, axis=1)
            boxes = zip(*boxes)

        labels = getattr(intrinsics, "labels", None)
        if not labels:
            labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat"]

        parsed = []
        for box, score, category in zip(boxes, scores, classes):
            conf_pct = float(score) * 100.0
            if conf_pct < min_conf_pct:
                continue

            cat_id = int(category)
            lbl_str = labels[cat_id].strip() if (0 <= cat_id < len(labels)) else f"class_{cat_id}"
            is_person = (lbl_str.lower() == "person" or cat_id == 0)

            if person_only and not is_person:
                continue

            coords = imx500.convert_inference_coords(box, metadata, picam2)
            x, y, w, h = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])

            if w < 28 or h < 40 or (is_person and w > h * 2.2):
                continue

            x = max(0, min(BINNED_W - 1, x))
            y = max(0, min(BINNED_H - 1, y))
            w = max(1, min(BINNED_W - x, w))
            h = max(1, min(BINNED_H - y, h))

            parsed.append({
                "box_2028": (x, y, w, h),
                "conf": conf_pct,
                "label": lbl_str.upper(),
                "is_person": is_person
            })

        parsed.sort(key=lambda d: d["conf"], reverse=True)
        if single_best_only and len(parsed) > 1:
            parsed = parsed[:1]
        elif len(parsed) > 3:
            parsed = parsed[:3]

        last_valid_detections = parsed
        return parsed
    except Exception:
        return last_valid_detections

# --- Ultra-Smooth Gimbal Zoom Scope State ---
zoom_state = "WIDE_SEARCH"
zoom_entered_time = 0.0
last_person_seen_time = 0.0
cand_lock_count = 0
last_cand_center = None
confirm_lock_count = 0
fps_last_time = time.time()
fps_val = 0.0

smooth_target_cx = BINNED_W * 0.5
smooth_target_cy = BINNED_H * 0.5
gimbal_x0 = float(BINNED_W * 0.3)
gimbal_y0 = float(BINNED_H * 0.3)
zoom_crop_2028 = [int(gimbal_x0), int(gimbal_y0), int(BINNED_W * 0.4), int(BINNED_H * 0.4)]

# --- Global Broadcast State ---
tx_active = False
tx_session_id = 0
tx_progress_idx = 0
tx_progress_total = 0
tx_pass = 0
last_trigger_time = 0.0
latest_tx_pil = Image.new("RGB", (TX_W, TX_H), (15, 23, 42))
latest_persons = []
capture_id = 0

def split_jpeg_header_and_scan(jpg_bytes):
    """
    Finds the SOS (0xFF 0xDA) marker in a JPEG and splits into (header_bytes, scan_body_bytes)
    at an EXACT multiple of 3 bytes so Base64(header) + Base64(scan) concatenates with zero '=' padding!
    """
    idx = 0
    n = len(jpg_bytes)
    while idx < n - 4:
        if jpg_bytes[idx] == 0xFF and jpg_bytes[idx + 1] == 0xDA:
            sos_len = (jpg_bytes[idx + 2] << 8) | jpg_bytes[idx + 3]
            raw_split = idx + 2 + sos_len
            # Round down to exact multiple of 3 bytes (e.g. 621 bytes = 207 * 3) so Base64 has NO '=' padding!
            split_pos = (raw_split // 3) * 3
            return jpg_bytes[:split_pos], jpg_bytes[split_pos:]
        idx += 1
    return b"", jpg_bytes

def broadcast_image_worker(pil_snapshot, max_conf, person_count, jpeg_quality, num_passes, mode_code, gps_str, note_str, target_box_2028, details_callback, my_session):
    global tx_active, tx_progress_idx, tx_progress_total, tx_pass, capture_id
    tx_active = True
    capture_id += 1
    cur_id = capture_id
    ts_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    short_time = datetime.datetime.now().strftime("%H:%M:%S")
    snap_w, snap_h = pil_snapshot.size

    is_scanline = mode_code in ("SCAN_TARGET", "SCAN_ROWS")
    target_first = (mode_code == "SCAN_TARGET")

    # Save local copy + details on the Raspberry Pi
    try:
        img_path = os.path.join(PI_LOG_DIR, f"SAR_Capture_{cur_id:03d}.jpg")
        pil_snapshot.save(img_path, format="JPEG", quality=jpeg_quality)
        with open(os.path.join(PI_LOG_DIR, "broadcast_history.log"), "a", encoding="utf-8") as lf:
            lf.write(f"[{ts_str}] Capture #{cur_id} | Res: {snap_w}x{snap_h} | Conf: {max_conf:.1f}% | Mode: {mode_code} | Model: {active_model_name} | GPS: {gps_str} | Note: {note_str} | File: {img_path}\n")
    except Exception as e:
        print(f"[Local Log Save Warning] {e}")

    # Proper inter-packet gap so the ESP32-S3 has time to write to 16MB Flash AND serve Wi-Fi to the phone without dropping packets!
    pkt_gap_s = max(0.055, LEVEL_CONFIGS[current_level][4] / 1000.0)

    try:
        if not is_scanline:
            # --- MODE: 2D Progressive Full-Frame JPEG (F:) ---
            buf = io.BytesIO()
            pil_snapshot.save(buf, format="JPEG", quality=jpeg_quality, optimize=True)
            jpg_bytes = buf.getvalue()
            full_b64 = base64.b64encode(jpg_bytes).decode("ascii").rstrip("=")

            chunk_size = 188
            chunks = [full_b64[i:i + chunk_size] for i in range(0, len(full_b64), chunk_size)]
            total_chunks = len(chunks)
            tx_progress_total = total_chunks

            details_callback(cur_id, ts_str, max_conf, mode_code, total_chunks, gps_str, note_str)

            det_pkt = f"D:{cur_id}:{max_conf:.1f}:{person_count}:{total_chunks}:{jpeg_quality}:{snap_w}:{snap_h}"
            safe_gps = gps_str.replace(":", ";")[:44]
            safe_note = f"[{active_model_name} {snap_w}x{snap_h}] " + note_str.replace(":", ";")[:32]
            meta_pkt = f"M:{cur_id}:{mode_code}:{short_time}:{safe_gps}:{safe_note}"

            for _ in range(3):
                if not tx_active or tx_session_id != my_session:
                    break
                lora_send_line(det_pkt)
                time.sleep(0.09)
                lora_send_line(meta_pkt)
                time.sleep(0.09)

            for p in range(1, num_passes + 1):
                if not tx_active or tx_session_id != my_session:
                    break
                tx_pass = p
                for idx, b64_chunk in enumerate(chunks):
                    if not tx_active or tx_session_id != my_session:
                        break
                    tx_progress_idx = idx + 1
                    chk2 = compute_checksum2(b64_chunk)
                    pkt = f"F:{idx}:{total_chunks}:{chk2}:{b64_chunk}"
                    lora_send_line(pkt)
                    time.sleep(pkt_gap_s)
        else:
            # --- MODE: 24-Bit 64x16 Picture Blocks (J:) - Multi-Resolution (320x240 up to 1280x960!) ---
            t_cols = snap_w // TILE_W
            t_rows = snap_h // TILE_H
            total_tiles = t_cols * t_rows
            tx_progress_total = total_tiles

            details_callback(cur_id, ts_str, max_conf, mode_code, total_tiles, gps_str, note_str)

            # Encode all 64x16 blocks at a common JPEG quality so they share common_hdr_b64;
            # if any single high-detail block (e.g. PiP border/text) exceeds 184 chars, lightly smooth just that block
            tile_q = min(int(jpeg_quality), 60)
            while True:
                tile_packets = []
                common_hdr_b64 = ""
                max_b64_len = 0
                for r in range(t_rows):
                    for c in range(t_cols):
                        x0, y0 = c * TILE_W, r * TILE_H
                        tile_img = pil_snapshot.crop((x0, y0, x0 + TILE_W, y0 + TILE_H))
                        for _blur_pass in range(4):
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

            if target_first and target_box_2028 is not None:
                bx, by, bw, bh = target_box_2028
                tcx = ((bx + bw * 0.5) / BINNED_W) * t_cols
                tcy = ((by + bh * 0.5) / BINNED_H) * t_rows
                tile_packets.sort(key=lambda item: (item[0] + 0.5 - tcx) ** 2 + (item[1] + 0.5 - tcy) ** 2)

            det_pkt = f"D:{cur_id}:{max_conf:.1f}:{person_count}:{total_tiles}:{tile_q}:{snap_w}:{snap_h}"
            safe_gps = gps_str.replace(":", ";")[:44]
            safe_note = f"[{active_model_name} {snap_w}x{snap_h}] " + note_str.replace(":", ";")[:32]
            meta_pkt = f"M:{cur_id}:{mode_code}:{short_time}:{safe_gps}:{safe_note}"

            hdr_chunks = [common_hdr_b64[i:i + 180] for i in range(0, len(common_hdr_b64), 180)]

            for _ in range(3):
                if not tx_active or tx_session_id != my_session:
                    break
                lora_send_line(det_pkt)
                time.sleep(0.08)
                lora_send_line(meta_pkt)
                time.sleep(0.08)
                for h_idx, h_part in enumerate(hdr_chunks):
                    lora_send_line(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                    time.sleep(0.07)

            for p in range(1, num_passes + 1):
                if not tx_active or tx_session_id != my_session:
                    break
                tx_pass = p
                # On rescan passes (p > 1), re-announce D:/M:/JH: headers, keep Target-First order so missed
                # blocks near the person repair first, and add +20ms packet spacing to avoid Wi-Fi poll collisions!
                if p > 1:
                    lora_send_line(det_pkt)
                    time.sleep(0.08)
                    lora_send_line(meta_pkt)
                    time.sleep(0.08)
                    for h_idx, h_part in enumerate(hdr_chunks):
                        lora_send_line(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                        time.sleep(0.07)

                pass_tiles = tile_packets
                pass_gap = pkt_gap_s + (0.020 if p > 1 else 0.0)

                for idx, (c, r, scan_b64) in enumerate(pass_tiles):
                    if not tx_active or tx_session_id != my_session:
                        break
                    tx_progress_idx = idx + 1
                    # Repeat D: + JH: header every 35 picture blocks so a phone/receiver joining mid-stream resizes & decodes immediately!
                    if idx > 0 and (idx % 35 == 0):
                        lora_send_line(det_pkt)
                        time.sleep(pass_gap)
                        for h_idx, h_part in enumerate(hdr_chunks):
                            lora_send_line(f"JH:{h_idx}:{len(hdr_chunks)}:{h_part}")
                            time.sleep(pass_gap)
                    chk2 = compute_checksum2(scan_b64)
                    # Encode t_cols (5, 10, 15, or 20) in J{t_cols}: so ESP32 knows exact resolution on every single block!
                    pkt = f"J{t_cols}:{c}:{r}:{chk2}:{scan_b64}"
                    lora_send_line(pkt)
                    time.sleep(pass_gap)

    except Exception as e:
        print(f"[Broadcast Worker Error] {e}")
    finally:
        if tx_session_id == my_session:
            tx_active = False
            try:
                root.after(0, lambda: btn_manual.config(text="Transmit Now", bg=C_GREEN, fg="#062118"))
            except Exception:
                pass

# --- Build Clean DJI Pilot 2 / Auterion Mission Control Ground Control GUI ---
BG_DARK   = "#111317"
BG_PANEL  = "#1A1D24"
BG_INPUT  = "#12151B"
BD_LINE   = "#2A2F3A"
C_BLUE    = "#2563EB"
C_BLUE_HI = "#3B82F6"
C_GREEN   = "#10B981"
C_AMBER   = "#F59E0B"
C_TEXT    = "#F3F4F6"
C_MUTED   = "#9CA3AF"
C_OK_BG   = "#0F2922"
C_OK_FG   = "#34D399"
C_RED     = "#EF4444"

F_TITLE   = ("DejaVu Sans", 8, "bold")
F_BOLD    = ("DejaVu Sans", 8, "bold")
F_SANS    = ("DejaVu Sans", 8)
F_SMALL   = ("DejaVu Sans", 7, "bold")
F_MONO    = ("Monospace", 8)

root = tk.Tk()
root.title("SAR Mission Control · IMX500 Optical & LoRa Telemetry")
root.configure(bg=BG_DARK)

main_frame = tk.Frame(root, bg=BG_DARK)
main_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

# 1. RIGHT COLUMN (Controls & Telemetry)
right_col = tk.Frame(main_frame, bg=BG_DARK)
right_col.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))

# 2. LEFT COLUMN (Stacked Optical Feeds: Wide on Top, Tracking Scope Below)
left_col = tk.Frame(main_frame, bg=BG_DARK)
left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# --- LEFT TOP: Wide Optical Feed ---
box_wide = tk.Frame(left_col, bg=BG_PANEL, padx=6, pady=5, highlightbackground=BD_LINE, highlightthickness=1)
box_wide.pack(fill=tk.X, pady=(0, 5))

wide_hdr_row = tk.Frame(box_wide, bg=BG_PANEL)
wide_hdr_row.pack(fill=tk.X, pady=(0, 3))
tk.Label(
    wide_hdr_row, text="Wide Optical Sensor · 2028×1520 (IMX500 NPU)",
    fg=C_TEXT, bg=BG_PANEL, font=F_TITLE
).pack(side=tk.LEFT)
lbl_fps_badge = tk.Label(
    wide_hdr_row, text="CAM 30 / UI 25 FPS · HB #0",
    fg=C_MUTED, bg=BG_INPUT, font=F_SMALL, padx=6, pady=1
)
lbl_fps_badge.pack(side=tk.RIGHT)

canvas_wide = tk.Canvas(box_wide, width=WIDE_DISP_W, height=WIDE_DISP_H, bg="#090B0E", highlightthickness=1, highlightbackground=BD_LINE)
canvas_wide.pack()
tk_wide_fb = tk.PhotoImage(width=WIDE_DISP_W, height=WIDE_DISP_H)
canvas_wide.create_image(0, 0, anchor=tk.NW, image=tk_wide_fb)

# --- LEFT BOTTOM: Target Tracking Scope ---
box_zoom = tk.Frame(left_col, bg=BG_PANEL, padx=6, pady=5, highlightbackground=BD_LINE, highlightthickness=1)
box_zoom.pack(fill=tk.X)

zoom_hdr_row = tk.Frame(box_zoom, bg=BG_PANEL)
zoom_hdr_row.pack(fill=tk.X, pady=(0, 3))
tk.Label(
    zoom_hdr_row, text="Target Tracking Scope (Auto-Opposite PiP)",
    fg=C_TEXT, bg=BG_PANEL, font=F_TITLE
).pack(side=tk.LEFT)
lbl_zoom_lock_badge = tk.Label(
    zoom_hdr_row, text="WIDE_SEARCH · LOCK 0/3",
    fg=C_MUTED, bg=BG_INPUT, font=F_SMALL, padx=6, pady=1
)
lbl_zoom_lock_badge.pack(side=tk.RIGHT)

canvas_zoom = tk.Canvas(box_zoom, width=ZOOM_DISP_W, height=ZOOM_DISP_H, bg="#090B0E", highlightthickness=1, highlightbackground=BD_LINE)
canvas_zoom.pack()
tk_zoom_fb = tk.PhotoImage(width=ZOOM_DISP_W, height=ZOOM_DISP_H)
canvas_zoom.create_image(0, 0, anchor=tk.NW, image=tk_zoom_fb)

lbl_tx_bar = tk.Label(
    left_col,
    text="LoRa Link Ready · 868.0 MHz",
    fg=C_MUTED, bg=BG_DARK, font=F_BOLD
)
lbl_tx_bar.pack(anchor=tk.W, pady=(4, 0))

# ==============================================================================
# RIGHT SIDE: MISSION CONTROL PANELS
# ==============================================================================

# --- PANEL 1: AI DETECTOR & PICTURE BLOCK FORMAT ---
panel_mode = tk.Frame(right_col, bg=BG_PANEL, padx=8, pady=6, highlightbackground=BD_LINE, highlightthickness=1)
panel_mode.pack(fill=tk.X, pady=(0, 5))

tk.Label(panel_mode, text="AI Detector & Picture Format", fg=C_MUTED, bg=BG_PANEL, font=F_TITLE).pack(anchor=tk.W)

model_btn_row = tk.Frame(panel_mode, bg=BG_PANEL)
model_btn_row.pack(fill=tk.X, pady=(3, 3))
model_buttons = {}

def select_ai_model_btn(m_name, m_path):
    if m_name == active_model_name:
        return
    for k, b in model_buttons.items():
        b.config(bg=C_BLUE if k == m_name else BG_INPUT, fg="#FFFFFF" if k == m_name else C_TEXT)
    def reload_worker():
        load_ai_model(m_name, m_path, do_vflip=vflip_var.get(), do_hflip=hflip_var.get())
        broadcast_config_to_receiver()
    threading.Thread(target=reload_worker, daemon=True).start()

for (m_name, m_path) in available_models:
    is_act = (m_name == active_model_name)
    b = tk.Button(
        model_btn_row, text=m_name,
        bg=C_BLUE if is_act else BG_INPUT,
        fg="#FFFFFF" if is_act else C_TEXT,
        relief=tk.FLAT, bd=1,
        font=F_SMALL, pady=2, padx=4,
        command=lambda mn=m_name, mp=m_path: select_ai_model_btn(mn, mp)
    )
    b.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
    model_buttons[m_name] = b

mode_btn_row = tk.Frame(panel_mode, bg=BG_PANEL)
mode_btn_row.pack(fill=tk.X, pady=(1, 4))
mode_buttons = {}

def select_tx_mode_btn(mode_code):
    global active_tx_mode
    active_tx_mode = mode_code
    for mc, mb in mode_buttons.items():
        mb.config(bg=C_BLUE if mc == mode_code else BG_INPUT, fg="#FFFFFF" if mc == mode_code else C_TEXT)
    broadcast_config_to_receiver()

for (m_code, m_lbl, _) in TX_MODES:
    is_act = (m_code == active_tx_mode)
    mb = tk.Button(
        mode_btn_row, text=m_lbl,
        bg=C_BLUE if is_act else BG_INPUT,
        fg="#FFFFFF" if is_act else C_TEXT,
        relief=tk.FLAT, bd=1,
        font=F_SMALL, pady=2, padx=3,
        command=lambda mc=m_code: select_tx_mode_btn(mc)
    )
    mb.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
    mode_buttons[m_code] = mb

res_btn_row = tk.Frame(panel_mode, bg=BG_PANEL)
res_btn_row.pack(fill=tk.X, pady=(0, 4))
res_buttons = {}

def select_tx_res_btn(res_code, w_px, h_px):
    global active_res_code, TX_W, TX_H, TILE_COLS, TILE_ROWS, tx_active, tx_session_id
    active_res_code = res_code
    TX_W, TX_H = w_px, h_px
    TILE_COLS = TX_W // TILE_W
    TILE_ROWS = TX_H // TILE_H
    # Stop any ongoing broadcast at the old resolution so it never overwrites the new resolution!
    tx_session_id += 1
    tx_active = False
    for rc, rb in res_buttons.items():
        rb.config(bg=C_BLUE if rc == res_code else BG_INPUT, fg="#FFFFFF" if rc == res_code else C_TEXT)
    def _res_sync():
        time.sleep(0.10)
        cfg_pkt = f"CFG:{current_level}:{active_tx_mode}:{active_model_name}:{thresh_var.get()}:{TX_W}:{TX_H}"
        for _ in range(4):
            lora_send_line(cfg_pkt)
            time.sleep(0.09)
    threading.Thread(target=_res_sync, daemon=True).start()

for (r_code, r_lbl, r_w, r_h) in TX_RESOLUTIONS:
    is_act = (r_code == active_res_code)
    rb = tk.Button(
        res_btn_row, text=r_lbl,
        bg=C_BLUE if is_act else BG_INPUT,
        fg="#FFFFFF" if is_act else C_TEXT,
        relief=tk.FLAT, bd=1,
        font=F_SMALL, pady=2, padx=2,
        command=lambda rc=r_code, rw=r_w, rh=r_h: select_tx_res_btn(rc, rw, rh)
    )
    rb.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
    res_buttons[r_code] = rb

lbl_live_conf = tk.Label(
    panel_mode,
    text="Target: Standby (0.0%)",
    fg=C_MUTED, bg=BG_INPUT, font=F_BOLD,
    padx=6, pady=3, width=34
)
lbl_live_conf.pack(fill=tk.X, pady=(0, 3))

chk_row1 = tk.Frame(panel_mode, bg=BG_PANEL)
chk_row1.pack(fill=tk.X)
single_box_var = tk.BooleanVar(value=True)
tk.Checkbutton(
    chk_row1, text="Single Target", variable=single_box_var,
    bg=BG_PANEL, fg=C_TEXT, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT, padx=(0, 4))

smart_zoom_var = tk.BooleanVar(value=True)
tk.Checkbutton(
    chk_row1, text="2-Stage Lock", variable=smart_zoom_var,
    bg=BG_PANEL, fg=C_TEXT, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT, padx=(0, 4))

person_only_var = tk.BooleanVar(value=True)
tk.Checkbutton(
    chk_row1, text="Person Only", variable=person_only_var,
    bg=BG_PANEL, fg=C_TEXT, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT)

chk_row2 = tk.Frame(panel_mode, bg=BG_PANEL)
chk_row2.pack(fill=tk.X)
vflip_var = tk.BooleanVar(value=True)
hflip_var = tk.BooleanVar(value=True)

def on_flip_toggle():
    for (m_name, m_path) in available_models:
        if m_name == active_model_name:
            def flip_worker():
                load_ai_model(m_name, m_path, do_vflip=vflip_var.get(), do_hflip=hflip_var.get())
            threading.Thread(target=flip_worker, daemon=True).start()
            break

tk.Checkbutton(
    chk_row2, text="V-Flip", variable=vflip_var, command=on_flip_toggle,
    bg=BG_PANEL, fg=C_MUTED, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT, padx=(0, 4))

tk.Checkbutton(
    chk_row2, text="H-Flip", variable=hflip_var, command=on_flip_toggle,
    bg=BG_PANEL, fg=C_MUTED, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT, padx=(0, 4))

auto_tx_var = tk.BooleanVar(value=True)
tk.Checkbutton(
    chk_row2, text="Auto-Send", variable=auto_tx_var,
    bg=BG_PANEL, fg=C_GREEN, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_BOLD
).pack(side=tk.LEFT, padx=(0, 4))

beacon_var = tk.BooleanVar(value=True)
tk.Checkbutton(
    chk_row2, text="Beacon", variable=beacon_var,
    bg=BG_PANEL, fg=C_MUTED, selectcolor=BG_INPUT, activebackground=BG_PANEL, font=F_SANS
).pack(side=tk.LEFT)

# --- PANEL 2: THRESHOLDS & RF MODULATION ---
panel_sliders = tk.Frame(right_col, bg=BG_PANEL, padx=8, pady=6, highlightbackground=BD_LINE, highlightthickness=1)
panel_sliders.pack(fill=tk.X, pady=(0, 5))

tk.Label(panel_sliders, text="Detection & Radio Link Settings", fg=C_MUTED, bg=BG_PANEL, font=F_TITLE).pack(anchor=tk.W)

s_grid = tk.Frame(panel_sliders, bg=BG_PANEL)
s_grid.pack(fill=tk.X, pady=(2, 0))
scol_l = tk.Frame(s_grid, bg=BG_PANEL)
scol_l.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
scol_r = tk.Frame(s_grid, bg=BG_PANEL)
scol_r.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

lbl_cand = tk.Label(scol_l, text="Wide Trigger: 38%", fg=C_TEXT, bg=BG_PANEL, font=F_SANS)
lbl_cand.pack(anchor=tk.W)
cand_var = tk.IntVar(value=38)
tk.Scale(
    scol_l, from_=20, to=85, orient=tk.HORIZONTAL, variable=cand_var,
    command=lambda v: lbl_cand.config(text=f"Wide Trigger: {int(float(v))}%"),
    bg=BG_PANEL, fg=C_TEXT, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_thresh = tk.Label(scol_r, text="Confirm Lock: 58%", fg=C_BLUE_HI, bg=BG_PANEL, font=F_BOLD)
lbl_thresh.pack(anchor=tk.W)
thresh_var = tk.IntVar(value=58)
tk.Scale(
    scol_r, from_=30, to=95, orient=tk.HORIZONTAL, variable=thresh_var,
    command=lambda v: lbl_thresh.config(text=f"Confirm Lock: {int(float(v))}%"),
    bg=BG_PANEL, fg=C_BLUE_HI, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_zoom_mag = tk.Label(scol_l, text="Scope Zoom: 2.5x", fg=C_TEXT, bg=BG_PANEL, font=F_SANS)
lbl_zoom_mag.pack(anchor=tk.W)
zoom_mag_var = tk.DoubleVar(value=2.5)
tk.Scale(
    scol_l, from_=1.5, to=4.0, resolution=0.1, orient=tk.HORIZONTAL, variable=zoom_mag_var,
    command=lambda v: lbl_zoom_mag.config(text=f"Scope Zoom: {float(v):.1f}x"),
    bg=BG_PANEL, fg=C_TEXT, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_cooldown = tk.Label(scol_r, text="Cooldown: 15s", fg=C_TEXT, bg=BG_PANEL, font=F_SANS)
lbl_cooldown.pack(anchor=tk.W)
cooldown_var = tk.IntVar(value=15)
tk.Scale(
    scol_r, from_=5, to=120, orient=tk.HORIZONTAL, variable=cooldown_var,
    command=lambda v: lbl_cooldown.config(text=f"Cooldown: {int(float(v))}s"),
    bg=BG_PANEL, fg=C_TEXT, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_jpg_q = tk.Label(scol_l, text="Image Quality: 62", fg=C_TEXT, bg=BG_PANEL, font=F_SANS)
lbl_jpg_q.pack(anchor=tk.W)
jpg_q_var = tk.IntVar(value=62)
tk.Scale(
    scol_l, from_=30, to=90, orient=tk.HORIZONTAL, variable=jpg_q_var,
    command=lambda v: lbl_jpg_q.config(text=f"Image Quality: {int(float(v))}"),
    bg=BG_PANEL, fg=C_TEXT, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_passes = tk.Label(scol_r, text="Rescan Passes: 2", fg=C_TEXT, bg=BG_PANEL, font=F_SANS)
lbl_passes.pack(anchor=tk.W)
passes_var = tk.IntVar(value=2)
tk.Scale(
    scol_r, from_=1, to=5, orient=tk.HORIZONTAL, variable=passes_var,
    command=lambda v: lbl_passes.config(text=f"Rescan Passes: {int(float(v))}"),
    bg=BG_PANEL, fg=C_TEXT, highlightthickness=0, troughcolor=BG_INPUT, showvalue=False, width=8
).pack(fill=tk.X)

lbl_lvl_name = tk.Label(panel_sliders, text=f"LoRa Preset: {LEVEL_CONFIGS[current_level][6]}", fg=C_MUTED, bg=BG_PANEL, font=F_SMALL)
lbl_lvl_name.pack(anchor=tk.W, pady=(3, 2))

lvl_btn_row = tk.Frame(panel_sliders, bg=BG_PANEL)
lvl_btn_row.pack(fill=tk.X)
level_buttons = {}

def select_lora_level_btn(new_lvl):
    if new_lvl == current_level:
        return
    for lk, lb in level_buttons.items():
        lb.config(bg=C_BLUE if lk == new_lvl else BG_INPUT, fg="#FFFFFF" if lk == new_lvl else C_TEXT)
    lbl_lvl_name.config(text=f"Syncing Receiver -> {LEVEL_CONFIGS[new_lvl][6]}...")
    def switch_worker():
        for _ in range(5):
            lora_send_line(f"CMD:LEVEL:{new_lvl}")
            time.sleep(0.12)
        set_lora_level(new_lvl)
        broadcast_config_to_receiver()
        root.after(0, lambda: lbl_lvl_name.config(text=f"LoRa Preset: {LEVEL_CONFIGS[new_lvl][6]}"))
    threading.Thread(target=switch_worker, daemon=True).start()

for lk in range(8):
    is_act = (lk == current_level)
    lb = tk.Button(
        lvl_btn_row, text=LEVEL_CONFIGS[lk][5],
        bg=C_BLUE if is_act else BG_INPUT,
        fg="#FFFFFF" if is_act else C_TEXT,
        relief=tk.FLAT, bd=1,
        font=F_SMALL, pady=2, padx=2,
        command=lambda lv=lk: select_lora_level_btn(lv)
    )
    lb.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
    level_buttons[lk] = lb

# --- PANEL 3: TELEMETRY & DISPATCH ---
panel_gps = tk.Frame(right_col, bg=BG_PANEL, padx=8, pady=6, highlightbackground=BD_LINE, highlightthickness=1)
panel_gps.pack(fill=tk.BOTH, expand=True)

tk.Label(panel_gps, text="Coordinates & Dispatch Control", fg=C_MUTED, bg=BG_PANEL, font=F_TITLE).pack(anchor=tk.W)

gps_row = tk.Frame(panel_gps, bg=BG_PANEL)
gps_row.pack(fill=tk.X, pady=(3, 2))
tk.Label(gps_row, text="GPS:", fg=C_MUTED, bg=BG_PANEL, font=F_BOLD).pack(side=tk.LEFT)
gps_var = tk.StringVar(value="51.5074 N, 0.1278 W")
ent_gps = tk.Entry(gps_row, textvariable=gps_var, bg=BG_INPUT, fg=C_AMBER, insertbackground="#FFFFFF", relief=tk.FLAT, font=F_MONO)
ent_gps.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

note_row = tk.Frame(panel_gps, bg=BG_PANEL)
note_row.pack(fill=tk.X, pady=(0, 4))
tk.Label(note_row, text="Unit:", fg=C_MUTED, bg=BG_PANEL, font=F_BOLD).pack(side=tk.LEFT)
note_var = tk.StringVar(value="SAR-1 // SECTOR ALPHA")
ent_note = tk.Entry(note_row, textvariable=note_var, bg=BG_INPUT, fg=C_TEXT, insertbackground="#FFFFFF", relief=tk.FLAT, font=F_MONO)
ent_note.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

txt_details = tk.Text(panel_gps, height=5, width=36, bg=BG_INPUT, fg=C_TEXT, font=F_MONO, highlightthickness=1, highlightbackground=BD_LINE, relief=tk.FLAT)
txt_details.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
txt_details.insert(tk.END, "Standby · Waiting for Target Lock\nMode : Target-First Picture Blocks\nFreq : 868.0 MHz (+22 dBm)")

def update_details_box(cid, ts_str, conf_val, mode_code, pkt_cnt, gps_s, note_s):
    mode_lbl = dict((c, l) for c, l, _ in TX_MODES).get(mode_code, mode_code)
    def _ui():
        txt_details.delete("1.0", tk.END)
        txt_details.insert(
            tk.END,
            f"Capture #{cid} · {ts_str}\n"
            f"Conf : {conf_val:.1f}%  |  {mode_lbl}\n"
            f"Blks : {pkt_cnt} ({active_model_name})\n"
            f"GPS  : {gps_s}\n"
            f"Unit : {note_s}"
        )
    root.after(0, _ui)

latest_wide_pil = Image.new("RGB", (WIDE_DISP_W, WIDE_DISP_H), (17, 19, 23))
latest_zoom_pil = Image.new("RGB", (ZOOM_DISP_W, ZOOM_DISP_H), (17, 19, 23))

# --- Dedicated Background Camera Capture Thread (Unlocks 25-30 FPS!) ---
latest_frame_2028 = None
latest_cam_wide_rgb = None
latest_cam_zoom_rgb = None
latest_cam_meta = None
latest_cam_crop = [int(gimbal_x0), int(gimbal_y0), int(BINNED_W * 0.4), int(BINNED_H * 0.4)]
cam_frame_seq = 0
cam_fps_val = 0.0
cam_last_ts = time.time()

def camera_capture_thread():
    global latest_frame_2028, latest_cam_wide_rgb, latest_cam_zoom_rgb, latest_cam_meta, latest_cam_crop, cam_frame_seq, cam_fps_val, cam_last_ts
    while True:
        if picam2 is None or cam_lock.locked():
            time.sleep(0.01)
            continue
        try:
            with cam_lock:
                if picam2 is None:
                    continue
                req = picam2.capture_request()
                frame_2028 = req.make_array("main")
                meta = req.get_metadata()
                req.release()

            zx0, zy0, zw, zh = zoom_crop_2028
            zx0 = max(0, min(BINNED_W - zw, zx0))
            zy0 = max(0, min(BINNED_H - zh, zy0))

            # Fast strided slices in background thread so Tkinter main thread never stalls
            w_sub = np.ascontiguousarray(frame_2028[::4, ::4, :3][:, :, ::-1])
            z_sub = np.ascontiguousarray(frame_2028[zy0:zy0 + zh:2, zx0:zx0 + zw:2, :3][:, :, ::-1])

            now_c = time.time()
            dt_c = now_c - cam_last_ts
            cam_last_ts = now_c
            if dt_c > 0:
                cam_fps_val = cam_fps_val * 0.85 + (1.0 / dt_c) * 0.15

            latest_frame_2028 = frame_2028
            latest_cam_wide_rgb = w_sub
            latest_cam_zoom_rgb = z_sub
            latest_cam_meta = meta
            latest_cam_crop = [zx0, zy0, zw, zh]
            cam_frame_seq += 1
        except Exception:
            time.sleep(0.01)

threading.Thread(target=camera_capture_thread, daemon=True).start()

def build_tx_snapshot_opposite_pip():
    """
    Composes the broadcast image at the selected resolution (320x240, 640x480, 960x720, or 1280x960)
    directly from the full 2028x1520 sensor array, and places the PiP zoom inset in the OPPOSITE corner from the person!
    """
    out_w, out_h = TX_W, TX_H
    pip_w = max(104, int(out_w * 0.33))
    pip_h = max(70, int(out_h * 0.29))

    if latest_frame_2028 is not None:
        try:
            full_rgb = np.ascontiguousarray(latest_frame_2028[:, :, :3][:, :, ::-1])
            pil_2028 = Image.fromarray(full_rgb, "RGB")
            tx_compose = pil_2028.resize((out_w, out_h), Image.BILINEAR)
            zx0, zy0, zw, zh = latest_cam_crop
            pip_zoom = pil_2028.crop((zx0, zy0, zx0 + zw, zy0 + zh)).resize((pip_w, pip_h), Image.BILINEAR)

            # Draw detection box on tx_compose
            draw_tx = ImageDraw.Draw(tx_compose)
            sx = out_w / float(BINNED_W)
            sy = out_h / float(BINNED_H)
            for p in latest_persons:
                bx, by, bw, bh = p["box_2028"]
                dx0, dy0 = int(bx * sx), int(by * sy)
                dx1, dy1 = int((bx + bw) * sx), int((by + bh) * sy)
                draw_tx.rectangle([dx0, dy0, dx1, dy1], outline=(16, 185, 129), width=max(2, out_w // 320))
                draw_tx.text((dx0 + 4, max(4, dy0 - 14)), f"TGT {p['conf']:.0f}%", fill=(16, 185, 129))
        except Exception:
            tx_compose = latest_wide_pil.resize((out_w, out_h), Image.BILINEAR)
            pip_zoom = latest_zoom_pil.resize((pip_w, pip_h), Image.BILINEAR)
    else:
        tx_compose = latest_wide_pil.resize((out_w, out_h), Image.BILINEAR)
        pip_zoom = latest_zoom_pil.resize((pip_w, pip_h), Image.BILINEAR)

    # Determine opposite corner from detected person so PiP NEVER covers the target
    if len(latest_persons) > 0:
        bx, by, bw, bh = latest_persons[0]["box_2028"]
        cx = bx + bw * 0.5
        cy = by + bh * 0.5
    else:
        zx0, zy0, zw, zh = zoom_crop_2028
        cx = zx0 + zw * 0.5
        cy = zy0 + zh * 0.5

    bar_h = max(18, int(out_h * 0.042))
    pip_x = 6 if (cx >= BINNED_W * 0.5) else (out_w - pip_w - 6)
    pip_y = 6 if (cy >= BINNED_H * 0.5) else (out_h - pip_h - bar_h - 4)

    tx_compose.paste(pip_zoom, (pip_x, pip_y))
    draw_tx = ImageDraw.Draw(tx_compose)
    draw_tx.rectangle([pip_x, pip_y, pip_x + pip_w, pip_y + pip_h], outline=(59, 130, 246), width=2)
    draw_tx.rectangle([0, out_h - bar_h, out_w, out_h], fill=(17, 19, 23))
    max_c = max([p["conf"] for p in latest_persons], default=0.0)
    draw_tx.text((6, out_h - bar_h + 3), f"CONF:{max_c:.0f}% | {out_w}x{out_h} | {gps_var.get()[:36]}", fill=(243, 244, 246))
    return tx_compose

def trigger_broadcast_now():
    global last_trigger_time, latest_tx_pil, tx_active, tx_session_id
    # Cancel any in-progress broadcast so manual Transmit Now always starts immediately at the selected resolution!
    tx_session_id += 1
    my_session = tx_session_id
    tx_active = True
    last_trigger_time = time.time()
    try:
        btn_manual.config(text="● BROADCASTING...", bg=C_AMBER, fg="#000000")
    except Exception:
        pass
    print(f"[TX] Manual Broadcast Triggered! Res: {TX_W}x{TX_H}, Mode: {active_tx_mode}")
    snap = build_tx_snapshot_opposite_pip()
    latest_tx_pil = snap
    max_c = max([p["conf"] for p in latest_persons], default=100.0)
    cnt = max(1, len(latest_persons))
    tbox = latest_persons[0]["box_2028"] if len(latest_persons) > 0 else None
    if auto_gps_fix:
        gps_var.set(auto_gps_fix)
    threading.Thread(
        target=broadcast_image_worker,
        args=(snap, max_c, cnt, jpg_q_var.get(), passes_var.get(), active_tx_mode, gps_var.get(), note_var.get(), tbox, update_details_box, my_session),
        daemon=True
    ).start()

def stop_broadcast_now():
    global tx_active, tx_session_id
    tx_session_id += 1
    tx_active = False

def clear_phone_screen_now():
    global tx_active, tx_session_id
    tx_session_id += 1
    tx_active = False
    def _clr_worker():
        time.sleep(0.08)
        for _ in range(4):
            lora_send_line("CMD:CLEAR")
            time.sleep(0.08)
    threading.Thread(target=_clr_worker, daemon=True).start()

btn_row = tk.Frame(panel_gps, bg=BG_PANEL)
btn_row.pack(fill=tk.X)

btn_manual = tk.Button(
    btn_row, text="Transmit Now", command=trigger_broadcast_now,
    bg=C_GREEN, fg="#062118", relief=tk.FLAT, font=F_BOLD, pady=4
)
btn_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))

btn_clear_phone = tk.Button(
    btn_row, text="Clear Phone", command=clear_phone_screen_now,
    bg="#262C38", fg=C_TEXT, relief=tk.FLAT, font=F_BOLD, pady=4
)
btn_clear_phone.pack(side=tk.LEFT, padx=2)

btn_stop = tk.Button(
    btn_row, text="Stop TX", command=stop_broadcast_now,
    bg=C_RED, fg="#FFFFFF", relief=tk.FLAT, font=F_BOLD, pady=4
)
btn_stop.pack(side=tk.LEFT, padx=(2, 0))

def pil_to_tk_canvas(pil_img, tk_target):
    w, h = pil_img.size
    ppm = f"P6 {w} {h} 255\n".encode("ascii") + pil_img.tobytes()
    tmp = tk.PhotoImage(data=ppm)
    tk_target.tk.call(tk_target, "copy", tmp, "-to", 0, 0)

last_rendered_seq = -1

def update_camera_and_ui():
    global latest_wide_pil, latest_zoom_pil, latest_persons, last_trigger_time, heartbeat_seq
    global zoom_state, zoom_entered_time, last_person_seen_time, cand_lock_count, last_cand_center, confirm_lock_count
    global smooth_target_cx, smooth_target_cy, gimbal_x0, gimbal_y0, zoom_crop_2028
    global fps_last_time, fps_val, last_rendered_seq

    t_start = time.time()
    if latest_cam_wide_rgb is None or cam_frame_seq == last_rendered_seq:
        root.after(5, update_camera_and_ui)
        return
    last_rendered_seq = cam_frame_seq

    cand_thresh = float(cand_var.get())
    confirm_thresh = float(thresh_var.get())
    zoom_mag = max(1.5, float(zoom_mag_var.get()))
    now = t_start

    if auto_gps_fix:
        gps_var.set(auto_gps_fix)

    dt = now - fps_last_time
    fps_last_time = now
    if dt > 0:
        fps_val = fps_val * 0.82 + (1.0 / dt) * 0.18

    wide_sub = latest_cam_wide_rgb
    zoom_sub = latest_cam_zoom_rgb
    metadata = latest_cam_meta
    zx0, zy0, zw, zh = latest_cam_crop

    dets = parse_detections_2028(
        metadata,
        min_conf_pct=cand_thresh,
        person_only=person_only_var.get(),
        single_best_only=single_box_var.get()
    )

    persons = [d for d in dets if d["is_person"]]
    latest_persons = persons
    max_conf = max([p["conf"] for p in persons], default=0.0)

    next_zw = max(280, min(BINNED_W, int(BINNED_W / zoom_mag)))
    next_zh = max(180, min(BINNED_H, int(next_zw * (ZOOM_DISP_H / float(ZOOM_DISP_W)))))

    # --- Smooth Gimbal Zoom Tracker ---
    if len(persons) > 0:
        last_person_seen_time = now
        bx, by, bw, bh = persons[0]["box_2028"]
        raw_cx = bx + bw * 0.5
        raw_cy = by + bh * 0.5

        err_dist = math.hypot(raw_cx - smooth_target_cx, raw_cy - smooth_target_cy)
        if err_dist > 22.0:
            smooth_target_cx = smooth_target_cx * 0.78 + raw_cx * 0.22
            smooth_target_cy = smooth_target_cy * 0.78 + raw_cy * 0.22
    elif (now - last_person_seen_time) > 2.5:
        smooth_target_cx = smooth_target_cx * 0.94 + (BINNED_W * 0.5) * 0.06
        smooth_target_cy = smooth_target_cy * 0.94 + (BINNED_H * 0.5) * 0.06

    desired_x0 = max(0.0, min(float(BINNED_W - next_zw), smooth_target_cx - next_zw * 0.5))
    desired_y0 = max(0.0, min(float(BINNED_H - next_zh), smooth_target_cy - next_zh * 0.5))

    gimbal_x0 += (desired_x0 - gimbal_x0) * 0.16
    gimbal_y0 += (desired_y0 - gimbal_y0) * 0.16
    zoom_crop_2028 = [int(gimbal_x0), int(gimbal_y0), next_zw, next_zh]

    # --- 2-Stage Search -> Zoom Confirmation State Machine ---
    if not smart_zoom_var.get():
        zoom_state = "WIDE_SEARCH"
    else:
        if zoom_state == "WIDE_SEARCH":
            if len(persons) > 0 and max_conf >= cand_thresh:
                bx, by, bw, bh = persons[0]["box_2028"]
                cx, cy = bx + bw // 2, by + bh // 2
                if last_cand_center is not None:
                    dist = abs(cx - last_cand_center[0]) + abs(cy - last_cand_center[1])
                    cand_lock_count = (cand_lock_count + 1) if dist < 320 else 1
                else:
                    cand_lock_count = 1
                last_cand_center = (cx, cy)

                if cand_lock_count >= 3:
                    zoom_state = "ZOOM_CONFIRM"
                    zoom_entered_time = now
                    confirm_lock_count = 1 if max_conf >= confirm_thresh else 0
                    cand_lock_count = 0
            else:
                cand_lock_count = max(0, cand_lock_count - 1)

        elif zoom_state == "ZOOM_CONFIRM":
            if len(persons) > 0 and max_conf >= confirm_thresh:
                confirm_lock_count += 1
            else:
                confirm_lock_count = max(0, confirm_lock_count - 1)

            if (now - zoom_entered_time) > 3.5 and confirm_lock_count < 3:
                zoom_state = "WIDE_SEARCH"
                confirm_lock_count = 0

    # --- Periodic 2.5-Second LoRa Heartbeat Beacon (Includes TX_W:TX_H so phone resolution is always synced!) ---
    if beacon_var.get() and (not tx_active) and (now - last_lora_tx_time) >= 2.5:
        heartbeat_seq += 1
        short_gps = gps_var.get().replace(":", ";")[:32]
        hb_pkt = f"H:{heartbeat_seq}:{zoom_state}:{max_conf:.1f}:{active_tx_mode}:{active_model_name}:{TX_W}:{TX_H}:{short_gps}"
        threading.Thread(target=lora_send_line, args=(hb_pkt,), daemon=True).start()

    # --- Fast Rendering of Stacked Wide & Scope Canvases ---
    pil_wide = Image.fromarray(wide_sub, "RGB").resize((WIDE_DISP_W, WIDE_DISP_H), Image.NEAREST)
    draw_wide = ImageDraw.Draw(pil_wide)

    scale_x = WIDE_DISP_W / float(BINNED_W)
    scale_y = WIDE_DISP_H / float(BINNED_H)

    wx0, wy0 = int(zx0 * scale_x), int(zy0 * scale_y)
    wx1, wy1 = int((zx0 + zw) * scale_x), int((zy0 + zh) * scale_y)
    scope_col = (59, 130, 246) if zoom_state == "ZOOM_CONFIRM" else (156, 163, 175)
    draw_wide.rectangle([wx0, wy0, wx1, wy1], outline=scope_col, width=2)
    draw_wide.text((wx0 + 4, max(4, wy0 - 14)), f"SCOPE {zoom_mag:.1f}X", fill=scope_col)

    for d in dets:
        bx, by, bw, bh = d["box_2028"]
        conf = d["conf"]
        dx0, dy0 = int(bx * scale_x), int(by * scale_y)
        dx1, dy1 = int((bx + bw) * scale_x), int((by + bh) * scale_y)
        col = (16, 185, 129) if conf >= confirm_thresh else (245, 158, 11)
        draw_wide.rectangle([dx0, dy0, dx1, dy1], outline=col, width=2)
        draw_wide.text((dx0 + 3, max(4, dy0 - 14)), f"TGT {conf:.0f}%", fill=col)

    pil_zoom_crop = Image.fromarray(zoom_sub, "RGB").resize((ZOOM_DISP_W, ZOOM_DISP_H), Image.NEAREST)
    draw_zoom = ImageDraw.Draw(pil_zoom_crop)

    z_scale_x = ZOOM_DISP_W / float(zw)
    z_scale_y = ZOOM_DISP_H / float(zh)

    for d in dets:
        bx, by, bw, bh = d["box_2028"]
        conf = d["conf"]
        zbx0 = int((bx - zx0) * z_scale_x)
        zby0 = int((by - zy0) * z_scale_y)
        zbx1 = int((bx + bw - zx0) * z_scale_x)
        zby1 = int((by + bh - zy0) * z_scale_y)
        if zbx1 > 0 and zby1 > 0 and zbx0 < ZOOM_DISP_W and zby0 < ZOOM_DISP_H:
            col = (16, 185, 129) if conf >= confirm_thresh else (245, 158, 11)
            draw_zoom.rectangle([zbx0, zby0, zbx1, zby1], outline=col, width=2)
            badge = f"TGT {conf:.0f}% [{min(confirm_lock_count, 3)}/3]"
            by_top = max(4, zby0 - 16)
            draw_zoom.rectangle([max(0, zbx0), by_top, max(0, zbx0) + len(badge) * 6 + 8, by_top + 14], fill=col)
            draw_zoom.text((max(0, zbx0) + 4, by_top + 1), badge, fill=(0, 0, 0))

    pil_to_tk_canvas(pil_wide, tk_wide_fb)
    pil_to_tk_canvas(pil_zoom_crop, tk_zoom_fb)

    latest_wide_pil = pil_wide
    latest_zoom_pil = pil_zoom_crop

    # --- Update Status Readouts & Auto-Broadcast Trigger ---
    lbl_fps_badge.config(text=f"CAM {cam_fps_val:.0f} / UI {fps_val:.0f} FPS · HB #{heartbeat_seq}")
    lbl_zoom_lock_badge.config(
        text=f"{zoom_state} · LOCK {min(confirm_lock_count, 3)}/3",
        bg=C_OK_BG if confirm_lock_count >= 2 else BG_INPUT,
        fg=C_OK_FG if confirm_lock_count >= 2 else C_MUTED
    )

    if len(persons) > 0 and max_conf >= confirm_thresh:
        lbl_live_conf.config(text=f"Locked Target: {max_conf:.1f}% ({min(confirm_lock_count, 3)}/3)", bg=C_OK_BG, fg=C_OK_FG)
    elif len(persons) > 0:
        lbl_live_conf.config(text=f"Acquiring Target: {max_conf:.1f}% ({cand_lock_count}/3)", bg="#2B2111", fg=C_AMBER)
    else:
        lbl_live_conf.config(text="Target: Standby (0.0%)", bg=BG_INPUT, fg=C_MUTED)

    cooldown_s = float(cooldown_var.get())
    if auto_tx_var.get() and (not tx_active) and (now - last_trigger_time) >= cooldown_s:
        if (zoom_state == "ZOOM_CONFIRM" and confirm_lock_count >= 3) or (not smart_zoom_var.get() and max_conf >= confirm_thresh):
            confirm_lock_count = 0
            trigger_broadcast_now()
            zoom_state = "WIDE_SEARCH"

    mode_lbl = dict((c, l) for c, l, _ in TX_MODES).get(active_tx_mode, active_tx_mode)
    if tx_active:
        pct = (tx_progress_idx / max(1, tx_progress_total)) * 100.0
        lbl_tx_bar.config(
            text=f"Transmitting · Pass {tx_pass} · Block {tx_progress_idx}/{tx_progress_total} ({pct:.0f}%) · {mode_lbl}",
            fg=C_BLUE_HI
        )
    else:
        rem_cool = max(0.0, cooldown_s - (now - last_trigger_time)) if last_trigger_time > 0 else 0.0
        if rem_cool > 0:
            lbl_tx_bar.config(text=f"Transmission Complete · Cooldown {rem_cool:.0f}s", fg=C_AMBER)
        else:
            lbl_tx_bar.config(text=f"Standby · {mode_lbl} · {active_model_name} · HB #{heartbeat_seq}", fg=C_MUTED)

    elapsed_ms = int((time.time() - t_start) * 1000)
    wait_ms = max(2, 28 - elapsed_ms)
    root.after(wait_ms, update_camera_and_ui)

root.after(150, update_camera_and_ui)
root.mainloop()
