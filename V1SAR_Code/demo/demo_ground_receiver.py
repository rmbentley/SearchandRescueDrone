#!/usr/bin/env python3
"""
========================================================================================
Search & Rescue (SAR) Drone System — Virtual Ground Receiver & Phone Web Server (DEMO)
========================================================================================
This script emulates the diymore / Heltec ESP32-S3 LoRa V4 hardware receiver.
It runs WITHOUT any hardware:
  1. Listens on a virtual LoRa RF channel (UDP port 9868) for Over-The-Air packets:
     - CFG:  (Configuration sync)
     - H:    (Drone heartbeat & live telemetry)
     - D:    (Target detection alert & resolution header)
     - M:    (Mission metadata & GPS)
     - JH:   (Common 64x16 JPEG header chunks)
     - J<cols>: (64x16 24-bit JPEG picture block payloads)
     - F:    (Full-frame progressive JPEG slices)
     - CMD:LEVEL: / CMD:CLEAR
  2. Verifies 12-bit polynomial CRC checksums (chk2) on incoming block payloads.
  3. Simulates front-end RF power (RSSI dBm and SNR dB) and 40-sample signal waveform.
  4. Hosts a local web server (http://localhost:8088 or local LAN IP for phones).
  5. Delivers the exact responsive DJI/Auterion-style mobile web HUD with:
     - Auto-resizing optical viewport (320x240 to 1280x960)
     - Real-time block streaming & Pass-2 missing-block reconciliation (/api/tiles?ids=...)
     - 5-bar signal meter, RSSI waveform graph, telemetry cards
     - Save Photo (JPG), Export Report (HTML), Signal CSV, and Remote Control
========================================================================================
"""

import sys
import os
import socket
import threading
import time
import json
import random
import base64
import urllib.parse
from http.server import HTTPServer, BaseHTTPRequestHandler

DEFAULT_HTTP_PORT = 8088
DEFAULT_LORA_UDP_PORT = 9868
DEFAULT_SENDER_UDP_PORT = 9869

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

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

# --- Shared Receiver State (Mirroring ESP32-S3 Global Variables) ---
state_lock = threading.RLock()

current_level = 5
LEVEL_NAMES = [
    "L0 · SF12 / 125k", "L1 · SF11 / 125k", "L2 · SF10 / 125k",
    "L3 · SF9 / 250k",  "L4 · SF8 / 250k",  "L5 · SF7 / 250k",
    "L6 · SF7 / 500k",  "L7 · SF6 / 500k"
]

det_capture_id = 0
det_person_count = 0
det_max_conf = 0.0
drone_live_conf = 0.0
heartbeat_seq = 0
drone_zoom_state = "STANDBY"
det_mode_str = "SCAN_TARGET"
det_model_str = "IMX500 SSD"
det_time_str = "--:--:--"
det_gps_str = "No Fix"
det_note_str = "SAR Demo Receiver"
det_jpg_quality = 60

img_width = 640
img_height = 480
tile_cols = 10
tile_rows = 30
active_num_tiles = 300

received_chunks = 0
total_chunks = 300
corrupted_packets = 0
total_packets_rx = 0
link_drop_count = 0
image_version = 0
global_tile_seq = 0
clear_seq = 0

last_packet_time = 0.0
last_rssi = -68.0
last_snr = 8.5
min_rssi = -68.0
max_rssi = -68.0

RSSI_HIST_LEN = 40
rssi_history = [-68.0] * 5

# Common JPEG header chunks (rebuilt from JH: packets)
common_hdr_chunks = {}
common_hdr_b64 = ""

# Picture block slots (index -> base64 scan string) and sequences (index -> seq)
MAX_TILES = 1200
tile_slots = [None] * MAX_TILES
tile_seqs = [0] * MAX_TILES

# Full frame chunks (F: mode)
full_frame_chunks = {}

def set_resolution_grid(w, h):
    global img_width, img_height, tile_cols, tile_rows, active_num_tiles
    with state_lock:
        img_width = w
        img_height = h
        tile_cols = w // 64
        tile_rows = h // 16
        active_num_tiles = tile_cols * tile_rows

def clear_receiver_state():
    global det_capture_id, received_chunks, total_chunks, image_version
    global global_tile_seq, clear_seq, tile_slots, tile_seqs
    global common_hdr_chunks, common_hdr_b64, full_frame_chunks
    global det_max_conf, det_person_count
    with state_lock:
        clear_seq += 1
        global_tile_seq = 0
        image_version = 0
        received_chunks = 0
        det_capture_id = 0
        total_chunks = 0
        det_max_conf = 0.0
        det_person_count = 0
        common_hdr_chunks.clear()
        common_hdr_b64 = ""
        full_frame_chunks.clear()
        tile_slots = [None] * MAX_TILES
        tile_seqs = [0] * MAX_TILES

def update_rssi_history(r_val):
    global rssi_history, min_rssi, max_rssi
    r_val = float(r_val)
    if len(rssi_history) == 0:
        min_rssi = r_val
        max_rssi = r_val
    else:
        if r_val < min_rssi: min_rssi = r_val
        if r_val > max_rssi: max_rssi = r_val
    rssi_history.append(r_val)
    if len(rssi_history) > RSSI_HIST_LEN:
        rssi_history.pop(0)

# --- Virtual LoRa RF Link Parser ---
def handle_lora_packet(line_str, simulated_rssi=None, simulated_snr=None):
    global det_capture_id, det_person_count, det_max_conf, drone_live_conf
    global heartbeat_seq, drone_zoom_state, det_mode_str, det_model_str
    global det_time_str, det_gps_str, det_note_str, det_jpg_quality
    global img_width, img_height, tile_cols, tile_rows, active_num_tiles
    global received_chunks, total_chunks, corrupted_packets, total_packets_rx
    global image_version, global_tile_seq, last_packet_time, last_rssi, last_snr
    global common_hdr_chunks, common_hdr_b64, current_level
    global tile_slots, tile_seqs, full_frame_chunks

    line_str = line_str.strip()
    if not line_str:
        return

    now = time.time()
    with state_lock:
        total_packets_rx += 1
        last_packet_time = now

        # Compute realistic RSSI/SNR if not provided by sender
        if simulated_rssi is not None:
            last_rssi = float(simulated_rssi)
        else:
            # Subtle natural drift
            last_rssi = max(-122.0, min(-45.0, last_rssi + random.uniform(-1.5, 1.5)))

        if simulated_snr is not None:
            last_snr = float(simulated_snr)
        else:
            last_snr = max(-15.0, min(12.0, (last_rssi + 95.0) / 4.0 + random.uniform(-0.5, 0.5)))

        update_rssi_history(last_rssi)

        # 1. Heartbeat Packet: H:<seq>:<zoomState>:<liveConf>:<txMode>:<aiModel>:<txW>:<txH>:<gpsStr>
        if line_str.startswith("H:"):
            parts = line_str.split(":", 8)
            if len(parts) >= 8:
                try:
                    heartbeat_seq = int(parts[1])
                    drone_zoom_state = parts[2]
                    drone_live_conf = float(parts[3])
                    det_mode_str = parts[4]
                    det_model_str = parts[5]
                    hw = int(parts[6])
                    hh = int(parts[7])
                    if len(parts) >= 9 and parts[8]:
                        det_gps_str = parts[8].replace(";", ", ")
                    if hw >= 320 and hh >= 240 and (hw != img_width or hh != img_height):
                        img_width, img_height = hw, hh
                        tile_cols = hw // 64
                        tile_rows = hh // 16
                        active_num_tiles = tile_cols * tile_rows
                except Exception:
                    pass
            return

        # 2. Config Sync Packet: CFG:<level>:<txMode>:<aiModel>:<confirmThresh>:<txW>:<txH>
        if line_str.startswith("CFG:"):
            parts = line_str.split(":")
            if len(parts) >= 7:
                try:
                    current_level = int(parts[1])
                    det_mode_str = parts[2]
                    det_model_str = parts[3]
                    cw = int(parts[5])
                    ch = int(parts[6])
                    if cw >= 320 and ch >= 240 and (cw != img_width or ch != img_height):
                        img_width, img_height = cw, ch
                        tile_cols = cw // 64
                        tile_rows = ch // 16
                        active_num_tiles = tile_cols * tile_rows
                        tile_slots = [None] * MAX_TILES
                        tile_seqs = [0] * MAX_TILES
                        global_tile_seq = 0
                except Exception:
                    pass
            return

        # 3. Detection Alert Header: D:<captureId>:<maxConfPct>:<personCount>:<totalUnits>:<jpegQ>:<txW>:<txH>
        if line_str.startswith("D:"):
            parts = line_str.split(":")
            if len(parts) >= 6:
                try:
                    cid = int(parts[1])
                    max_c = float(parts[2])
                    p_cnt = int(parts[3])
                    t_units = int(parts[4])
                    jq = int(parts[5])

                    if len(parts) >= 8:
                        dw = int(parts[6])
                        dh = int(parts[7])
                        if dw >= 320 and dh >= 240 and (dw != img_width or dh != img_height):
                            img_width, img_height = dw, dh
                            tile_cols = dw // 64
                            tile_rows = dh // 16
                            active_num_tiles = tile_cols * tile_rows

                    if cid != det_capture_id:
                        det_capture_id = cid
                        received_chunks = 0
                        common_hdr_chunks.clear()
                        common_hdr_b64 = ""
                        full_frame_chunks.clear()
                        tile_slots = [None] * MAX_TILES
                        tile_seqs = [0] * MAX_TILES

                    det_max_conf = max_c
                    det_person_count = p_cnt
                    total_chunks = t_units
                    det_jpg_quality = jq
                except Exception:
                    pass
            return

        # 4. Mission Metadata: M:<captureId>:<modeCode>:<HH;MM;SS>:<gpsStr>:<noteStr>
        if line_str.startswith("M:"):
            parts = line_str.split(":", 5)
            if len(parts) >= 6:
                try:
                    det_capture_id = int(parts[1])
                    det_mode_str = parts[2]
                    det_time_str = parts[3].replace(";", ":")
                    det_gps_str = parts[4].replace(";", ":")
                    det_note_str = parts[5].replace(";", ":")
                except Exception:
                    pass
            return

        # 5. Common JPEG Header: JH:<partIdx>:<totalParts>:<hdrB64Chunk>
        if line_str.startswith("JH:"):
            parts = line_str.split(":", 3)
            if len(parts) >= 4:
                try:
                    p_idx = int(parts[1])
                    tot_p = int(parts[2])
                    chunk = parts[3]
                    common_hdr_chunks[p_idx] = chunk
                    if len(common_hdr_chunks) == tot_p:
                        full_hdr = "".join(common_hdr_chunks[i] for i in range(tot_p))
                        common_hdr_b64 = full_hdr
                except Exception:
                    pass
            return

        # 6. Picture Block: J<cols>:<col>:<row>:<chk2>:<scanB64> or J:<col>:<row>:<chk2>:<scanB64>
        if line_str.startswith("J"):
            if det_capture_id == 0:
                return  # Standby / Cleared: reject orphaned blocks until new D: header
            first_colon = line_str.find(":")
            if first_colon > 0:
                pfx = line_str[:first_colon]
                t_cols_hint = 0
                if len(pfx) > 1 and pfx[1:].isdigit():
                    t_cols_hint = int(pfx[1:])

                parts = line_str[first_colon + 1:].split(":", 3)
                if len(parts) >= 4:
                    try:
                        c = int(parts[0])
                        r = int(parts[1])
                        chk2 = parts[2]
                        scan_b64 = parts[3]

                        # Verify 12-bit checksum
                        expected_chk = compute_checksum2(scan_b64)
                        if chk2 != expected_chk:
                            corrupted_packets += 1
                            return

                        # Auto-sync resolution grid from t_cols_hint if needed
                        if t_cols_hint in (5, 10, 15, 20) and t_cols_hint != tile_cols:
                            img_width = t_cols_hint * 64
                            img_height = (t_cols_hint * 3) * 16
                            tile_cols = t_cols_hint
                            tile_rows = t_cols_hint * 3
                            active_num_tiles = tile_cols * tile_rows

                        tile_idx = r * tile_cols + c
                        if 0 <= tile_idx < MAX_TILES:
                            is_new = (tile_slots[tile_idx] is None)
                            tile_slots[tile_idx] = scan_b64
                            global_tile_seq += 1
                            tile_seqs[tile_idx] = global_tile_seq
                            if is_new:
                                received_chunks += 1
                    except Exception:
                        pass
            return

        # 7. Full-Frame JPEG Slice: F:<chunkIdx>:<totalChunks>:<chk2>:<b64Chunk>
        if line_str.startswith("F:"):
            if det_capture_id == 0:
                return  # Standby / Cleared: reject orphaned slices until new D: header
            parts = line_str.split(":", 4)
            if len(parts) >= 5:
                try:
                    c_idx = int(parts[1])
                    tot_c = int(parts[2])
                    chk2 = parts[3]
                    b64_c = parts[4]

                    if compute_checksum2(b64_c) != chk2:
                        corrupted_packets += 1
                        return

                    is_new = (c_idx not in full_frame_chunks)
                    full_frame_chunks[c_idx] = b64_c
                    if is_new:
                        received_chunks += 1
                    total_chunks = tot_c
                    if len(full_frame_chunks) == tot_c:
                        image_version += 1
                except Exception:
                    pass
            return

        # 8. Remote Level Command: CMD:LEVEL:<lvl>
        if line_str.startswith("CMD:LEVEL:"):
            try:
                current_level = int(line_str.split(":")[2])
            except Exception:
                pass
            return

        # 9. Remote Clear Command: CMD:CLEAR
        if line_str == "CMD:CLEAR":
            clear_receiver_state()
            return

# --- Web Server Request Handler (Mirroring ESP32-S3 Web Endpoints) ---
INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>SAR Ground Receiver (Virtual Hardware Demo)</title>
<style>
  :root {
    --bg: #111317;
    --card: #1a1d24;
    --card-alt: #212630;
    --border: #2a2f3a;
    --blue: #2563eb;
    --blue-hi: #3b82f6;
    --green: #10b981;
    --amber: #f59e0b;
    --red: #ef4444;
    --text: #f3f4f6;
    --sub: #9ca3af;
  }
  * {
    box-sizing: border-box; margin: 0; padding: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-variant-numeric: tabular-nums;
  }
  html, body { width: 100%; max-width: 100%; overflow-x: hidden; background: var(--bg); color: var(--text); font-size: 13px; }
  body { padding: 8px; max-width: 720px; margin: 0 auto; }

  /* Top Status Bar */
  .header {
    display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;
    background: var(--card); border: 1px solid var(--border); border-radius: 8px;
    padding: 8px 12px; margin-bottom: 8px;
  }
  .brand { display: flex; align-items: center; gap: 8px; font-weight: 600; font-size: 14px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--green); box-shadow: 0 0 8px var(--green); }
  .dot.offline { background: var(--red); box-shadow: 0 0 8px var(--red); }
  .rf-pills { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
  .pill {
    background: var(--card-alt); border: 1px solid var(--border); border-radius: 6px;
    padding: 3px 8px; font-size: 11px; font-weight: 600; color: var(--text); white-space: nowrap;
  }
  .pill span { color: var(--sub); font-weight: 500; margin-right: 3px; }
  .sig-bars { display: inline-flex; align-items: flex-end; gap: 2px; height: 12px; margin-right: 4px; vertical-align: middle; }
  .sig-bars i { width: 3px; background: #374151; border-radius: 1px; display: inline-block; }
  .sig-bars i.on { background: var(--green); }
  .sig-bars.warn i.on { background: var(--amber); }
  .sig-bars.bad i.on { background: var(--red); }

  /* Video Viewport with Translucent Glass HUD Overlays */
  .viewport-wrap {
    position: relative; width: 100%; aspect-ratio: 4/3; background: #090b0e;
    border: 1px solid var(--border); border-radius: 8px; overflow: hidden; margin-bottom: 8px;
  }
  #viewCanvas { width: 100%; height: 100%; display: block; }
  .hud-top {
    position: absolute; top: 8px; left: 8px; right: 8px;
    display: flex; justify-content: space-between; align-items: flex-start; gap: 6px; pointer-events: none;
  }
  .hud-bottom {
    position: absolute; bottom: 10px; left: 8px; right: 8px;
    display: flex; justify-content: space-between; align-items: center; gap: 6px; pointer-events: none;
  }
  .hud-chip {
    background: rgba(15, 18, 24, 0.78); backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px);
    border: 1px solid rgba(255, 255, 255, 0.12); border-radius: 6px;
    padding: 4px 9px; font-size: 11px; font-weight: 600; color: #f9fafb;
    max-width: 100%; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
  }
  .hud-chip b { color: #60a5fa; }
  .hud-chip.amber b { color: #fbbf24; }
  .prog-track {
    position: absolute; bottom: 0; left: 0; right: 0; height: 4px; background: rgba(255, 255, 255, 0.1);
  }
  .prog-fill {
    height: 100%; width: 0%; background: var(--blue-hi); transition: width 0.18s ease-out;
  }

  /* Two-Column Telemetry & Signal Graph Grid */
  .grid-2 { display: grid; grid-template-columns: 1fr; gap: 8px; margin-bottom: 8px; }
  @media (min-width: 580px) {
    .grid-2 { grid-template-columns: 1.2fr 1fr; }
  }
  .card {
    background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; min-width: 0;
  }
  .card-title {
    font-size: 11px; font-weight: 600; color: var(--sub); text-transform: uppercase; letter-spacing: 0.4px;
    display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;
  }
  .info-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px 10px; }
  .info-item { min-width: 0; }
  .info-label { font-size: 10px; color: var(--sub); margin-bottom: 1px; }
  .info-val { font-size: 12px; font-weight: 600; color: var(--text); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  #rssiCanvas { width: 100%; height: 76px; background: #12151b; border: 1px solid var(--border); border-radius: 6px; display: block; }

  /* Action Toolbar */
  .toolbar {
    display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 6px; margin-bottom: 8px;
  }
  @media (min-width: 580px) {
    .toolbar { grid-template-columns: repeat(6, minmax(0, 1fr)); }
  }
  button, select, a.btn {
    background: var(--card-alt); color: var(--text); border: 1px solid var(--border); border-radius: 6px;
    padding: 8px 8px; font-size: 11px; font-weight: 600; cursor: pointer; text-decoration: none;
    text-align: center; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; transition: background 0.12s;
  }
  button:hover, a.btn:hover { background: #2b313e; }
  button:active, a.btn:active { transform: translateY(1px); }
  button.primary { background: var(--blue); border-color: var(--blue-hi); color: #fff; }
  button.danger { background: rgba(239, 68, 68, 0.16); border-color: rgba(239, 68, 68, 0.45); color: #fca5a5; }

  /* Capture History List */
  .log-list { max-height: 125px; overflow-y: auto; font-size: 11px; }
  .log-row {
    display: flex; justify-content: space-between; align-items: center; gap: 8px;
    padding: 6px 0; border-bottom: 1px solid var(--border); min-width: 0;
  }
  .log-row:last-child { border-bottom: none; }
  .log-row span { min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: var(--sub); }
  .log-row span b { color: var(--text); }
</style>
</head>
<body>

  <!-- Top Status Bar -->
  <div class="header">
    <div class="brand">
      <span class="dot" id="linkDot"></span>
      <span id="linkTitle">SAR Virtual Ground Link · 868 MHz</span>
    </div>
    <div class="rf-pills">
      <div class="pill">
        <span class="sig-bars" id="sigBars"><i style="height:3px"></i><i style="height:5px"></i><i style="height:7px"></i><i style="height:10px"></i><i style="height:12px"></i></span>
        <span>RSSI</span><b id="topRssi">-- dBm</b>
      </div>
      <div class="pill"><span>SNR</span><b id="topSnr">-- dB</b></div>
      <div class="pill"><span>Quality</span><b id="topQual">--%</b></div>
    </div>
  </div>

  <!-- Primary 4:3 Optical Viewport with Glass HUD Overlays -->
  <div class="viewport-wrap">
    <canvas id="viewCanvas" width="640" height="480"></canvas>
    <div class="hud-top">
      <div class="hud-chip amber" id="hudTarget">Capture #0 · <b>Standby</b></div>
      <div class="hud-chip" id="hudBlocks">Blocks: <b>0 / 300 (0%)</b></div>
    </div>
    <div class="hud-bottom">
      <div class="hud-chip" id="hudGps">GPS: <b>Waiting for telemetry...</b></div>
      <div class="hud-chip" id="hudMode">Mode: <b>Target-First Blocks</b></div>
    </div>
    <div class="prog-track"><div class="prog-fill" id="progBar"></div></div>
  </div>

  <!-- Telemetry & Signal History Cards -->
  <div class="grid-2">
    <div class="card">
      <div class="card-title">
        <span>Capture & Link Telemetry</span>
        <span id="capTimeBadge">--:--:--</span>
      </div>
      <div class="info-grid">
        <div class="info-item">
          <div class="info-label">Drone State</div>
          <div class="info-val" id="valState">Standby</div>
        </div>
        <div class="info-item">
          <div class="info-label">AI Detector</div>
          <div class="info-val" id="valModel">IMX500 SSD</div>
        </div>
        <div class="info-item">
          <div class="info-label">Unit / Sector</div>
          <div class="info-val" id="valNote">--</div>
        </div>
        <div class="info-item">
          <div class="info-label">Packets & CRC</div>
          <div class="info-val" id="valDiag">0 pkts · 0 CRC · 0 KB</div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">
        <span>Signal Power History (dBm)</span>
        <span id="ageSpan">No packets yet</span>
      </div>
      <canvas id="rssiCanvas" width="340" height="76"></canvas>
    </div>
  </div>

  <!-- Action Toolbar -->
  <div class="toolbar">
    <button class="primary" onclick="saveCanvasJpg()">Save Photo</button>
    <button onclick="saveFullReportHtml()">Export Report</button>
    <button onclick="saveRssiCsvLog()">Signal CSV</button>
    <button class="danger" onclick="clearPhoneImage()">Clear Screen</button>
    <select id="resSel" onchange="changeResolution(this.value)">
      <option value="320x240">320×240 (75 blk)</option>
      <option value="640x480" selected>640×480 (300 blk)</option>
      <option value="960x720">960×720 (675 blk)</option>
      <option value="1280x960">1280×960 (1200 blk)</option>
    </select>
    <select id="lvlSel" onchange="changeLevel(this.value)">
      <option value="0">L0 · SF12 / 125k</option>
      <option value="1">L1 · SF11 / 125k</option>
      <option value="2">L2 · SF10 / 125k</option>
      <option value="3">L3 · SF9 / 250k</option>
      <option value="4">L4 · SF8 / 250k</option>
      <option value="5" selected>L5 · SF7 / 250k</option>
      <option value="6">L6 · SF7 / 500k</option>
      <option value="7">L7 · SF6 / 500k</option>
    </select>
  </div>

  <!-- Saved Captures Log -->
  <div class="card">
    <div class="card-title">
      <span>Saved Captures</span>
      <div style="display:flex; gap:10px; align-items:center;">
        <label style="cursor:pointer; font-size:11px; color:var(--text); text-transform:none;">
          <input type="checkbox" id="autoSaveChk" checked style="vertical-align:middle;"> Auto-Save (≥90%)
        </label>
        <button onclick="clearSavedHistory()" style="padding:2px 8px; font-size:10px;">Clear Log</button>
      </div>
    </div>
    <div class="log-list" id="savedList"></div>
  </div>

<script>
const vCanvas = document.getElementById('viewCanvas');
const vCtx = vCanvas.getContext('2d');
const rCanvas = document.getElementById('rssiCanvas');
const rCtx = rCanvas.getContext('2d');

let lastCaptureId = -1;
let lastImgVer = 0;
let lastTileSeq = 0;
let lastClearSeq = -1;
let latestStatus = {};
let rssiLogRows = [];
let savedCaptures = [];
let lastAutoLoggedCaptureId = -1;
let curImgW = 640;
let curImgH = 480;
let curTileCols = 10;
let curTileRows = 30;
let drawnTiles = new Array(300).fill(false);
let drawnCount = 0;
let missingCursor = 0;
let isFetchingTiles = false;

function formatModeLabel(m) {
  if (!m) return 'Target-First Blocks';
  if (m === 'SCAN_TARGET') return 'Target-First Blocks';
  if (m === 'SCAN_ROWS') return 'Row-Order Blocks';
  if (m === '2D_FULL') return 'Full-Frame Packets';
  return m;
}

function resetDrawnTiles() {
  drawnTiles = new Array(curTileCols * curTileRows).fill(false);
  drawnCount = 0;
  missingCursor = 0;
}

function drawBlankCanvas(msg) {
  const w = vCanvas.width, h = vCanvas.height;
  vCtx.fillStyle = '#090b0e';
  vCtx.fillRect(0, 0, w, h);

  vCtx.strokeStyle = '#141821';
  vCtx.lineWidth = 1;
  vCtx.beginPath();
  for (let x = 64; x < w; x += 64) {
    vCtx.moveTo(x, 0); vCtx.lineTo(x, h);
  }
  for (let y = 16; y < h; y += 16) {
    vCtx.moveTo(0, y); vCtx.lineTo(w, y);
  }
  vCtx.stroke();

  vCtx.strokeStyle = '#252c3b';
  vCtx.beginPath();
  const ch = Math.max(16, Math.round(w * 0.04));
  vCtx.moveTo(w / 2, h / 2 - ch); vCtx.lineTo(w / 2, h / 2 + ch);
  vCtx.moveTo(w / 2 - ch, h / 2); vCtx.lineTo(w / 2 + ch, h / 2);
  vCtx.strokeRect(Math.round(w * 0.075), Math.round(h * 0.075), Math.round(w * 0.85), Math.round(h * 0.85));
  vCtx.stroke();

  const fSize = Math.max(11, Math.round(w * 0.022));
  vCtx.fillStyle = '#9ca3af';
  vCtx.font = `600 ${fSize}px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif`;
  vCtx.textAlign = 'center';
  vCtx.fillText(`${curImgW}×${curImgH} Viewport · ${curTileCols}×${curTileRows} Grid (${curTileCols * curTileRows} Blocks)`, w / 2, h / 2 - ch - 10);
  vCtx.fillStyle = '#6b7280';
  vCtx.fillText(msg || 'Optical Feed Standby · Waiting for Drone Transmission', w / 2, h / 2 + ch + fSize + 8);
  vCtx.textAlign = 'left';
}
drawBlankCanvas();

function applyCanvasResolution(w, h, reasonMsg) {
  w = parseInt(w, 10);
  h = parseInt(h, 10);
  if (!w || !h || w < 320 || h < 240) return;
  if (w === curImgW && h === curImgH && vCanvas.width === w && vCanvas.height === h) return;
  curImgW = w;
  curImgH = h;
  curTileCols = Math.round(w / 64);
  curTileRows = Math.round(h / 16);
  vCanvas.width = w;
  vCanvas.height = h;
  const resSel = document.getElementById('resSel');
  if (resSel) resSel.value = `${w}x${h}`;
  lastTileSeq = 0;
  resetDrawnTiles();
  drawBlankCanvas(reasonMsg || `Viewport Adjusted to ${w}×${h} (${curTileCols * curTileRows} Blocks)`);
}

async function clearPhoneImage() {
  lastTileSeq = 0;
  lastImgVer = 0;
  lastCaptureId = 0;
  resetDrawnTiles();
  drawBlankCanvas('Screen Cleared · Standby');
  document.getElementById('progBar').style.width = '0%';
  document.getElementById('hudBlocks').innerHTML = `${curImgW}×${curImgH} Viewport · <b>Standby</b>`;
  document.getElementById('hudTarget').innerHTML = `Optical Feed · <b>Standby</b>`;
  try {
    await fetch('/api/clear');
  } catch (e) {}
}

function clearSavedHistory() {
  savedCaptures = [];
  try { localStorage.removeItem('sar_saved_captures'); } catch (e) {}
  renderSavedHistoryList();
}

function updateSignalIndicators(rssi, snr, totalPkts, dropped) {
  const rssiEl = document.getElementById('topRssi');
  const snrEl = document.getElementById('topSnr');
  const qualEl = document.getElementById('topQual');
  const barsEl = document.getElementById('sigBars');
  const bars = barsEl.querySelectorAll('i');

  if (!totalPkts || totalPkts === 0) {
    rssiEl.textContent = '-- dBm';
    snrEl.textContent = '-- dB';
    qualEl.textContent = '--%';
    bars.forEach(b => b.className = '');
    return;
  }

  const qual = Math.max(0, Math.min(100, Math.round(((rssi + 120) / 90) * 100)));
  const activeBars = dropped ? 0 : Math.max(1, Math.min(5, Math.ceil(qual / 20)));

  rssiEl.textContent = `${rssi.toFixed(0)} dBm`;
  snrEl.textContent = `${snr >= 0 ? '+' : ''}${snr.toFixed(1)} dB`;
  qualEl.textContent = `${qual}%`;

  barsEl.className = 'sig-bars' + (qual < 35 ? ' bad' : (qual < 65 ? ' warn' : ''));
  bars.forEach((b, idx) => {
    b.className = (idx < activeBars) ? 'on' : '';
  });
}

function drawRssiChart(hist, dropped) {
  const w = rCanvas.width, h = rCanvas.height;
  rCtx.fillStyle = '#12151b';
  rCtx.fillRect(0, 0, w, h);

  rCtx.strokeStyle = '#1f2430';
  rCtx.lineWidth = 1;
  [14, h / 2, h - 12].forEach(y => {
    rCtx.beginPath();
    rCtx.moveTo(38, y);
    rCtx.lineTo(w - 6, y);
    rCtx.stroke();
  });

  rCtx.fillStyle = '#6b7280';
  rCtx.font = '10px -apple-system, sans-serif';
  rCtx.fillText('-30', 6, 17);
  rCtx.fillText('-75', 6, h / 2 + 3);
  rCtx.fillText('-120', 4, h - 9);

  if (!hist || hist.length === 0) return;

  const plotX0 = 40, plotW = w - 48;
  const strokeCol = dropped ? '#ef4444' : '#3b82f6';
  rCtx.strokeStyle = strokeCol;
  rCtx.lineWidth = 2;
  rCtx.beginPath();
  for (let i = 0; i < hist.length; i++) {
    const x = plotX0 + (i / Math.max(1, 39)) * plotW;
    const norm = Math.max(0, Math.min(1, (hist[i] + 120) / 90.0));
    const y = (h - 12) - norm * (h - 26);
    if (i === 0) rCtx.moveTo(x, y);
    else rCtx.lineTo(x, y);
  }
  rCtx.stroke();
}

function joinB64Safe(hdrB64, scanB64) {
  if (!hdrB64 || !scanB64) return '';
  let full = hdrB64.replace(/=+$/, '') + scanB64.replace(/=+$/, '');
  const pad = full.length % 4;
  if (pad === 2) full += '==';
  else if (pad === 3) full += '=';
  return full;
}

function drawTileBatch(hdr, tiles) {
  return new Promise(resolve => {
    if (!hdr || !tiles || tiles.length === 0) {
      resolve();
      return;
    }
    let pending = tiles.length;
    const doneOne = () => {
      pending--;
      if (pending <= 0) resolve();
    };
    const totalGrid = curTileCols * curTileRows;
    for (const t of tiles) {
      const idx = t.r * curTileCols + t.c;
      const fullB64 = joinB64Safe(hdr, t.b64);
      const img = new Image();
      img.onload = () => {
        if (drawnCount === 0) {
          vCtx.fillStyle = '#090b0e';
          vCtx.fillRect(0, 0, vCanvas.width, vCanvas.height);
        }
        vCtx.drawImage(img, t.c * 64, t.r * 16, 64, 16);
        if (idx >= 0 && idx < totalGrid && !drawnTiles[idx]) {
          drawnTiles[idx] = true;
          drawnCount++;
        }
        doneOne();
      };
      img.onerror = doneOne;
      img.src = 'data:image/jpeg;base64,' + fullB64;
    }
  });
}

async function fetchPictureBlocks(sinceSeq) {
  if (isFetchingTiles) return;
  isFetchingTiles = true;
  try {
    let fromIdx = 0;
    let firstPageMaxSeq = 0;
    while (true) {
      const ctrl = new AbortController();
      const tid = setTimeout(() => ctrl.abort(), 1800);
      const r = await fetch(`/api/tiles?since=${sinceSeq}&from=${fromIdx}`, {signal: ctrl.signal});
      clearTimeout(tid);
      const d = await r.json();
      if (d.w && d.h) {
        applyCanvasResolution(d.w, d.h);
      }
      if (firstPageMaxSeq === 0 && d.maxSeq) {
        firstPageMaxSeq = d.maxSeq;
      }
      if (!d.hdr || !d.tiles || d.tiles.length === 0) {
        break;
      }
      await drawTileBatch(d.hdr, d.tiles);
      if (d.more && typeof d.nextFrom === 'number') {
        fromIdx = d.nextFrom;
      } else {
        break;
      }
    }
    if (firstPageMaxSeq > lastTileSeq) {
      lastTileSeq = firstPageMaxSeq;
    }
  } catch (e) {
  } finally {
    isFetchingTiles = false;
  }
}

async function reconcileMissingBlocks() {
  if (isFetchingTiles) return;
  const totalGrid = curTileCols * curTileRows;
  if (drawnCount >= totalGrid) return;
  const missingIds = [];
  for (let step = 0; step < totalGrid && missingIds.length < 25; step++) {
    const idx = (missingCursor + step) % totalGrid;
    if (!drawnTiles[idx]) {
      missingIds.push(idx);
    }
  }
  missingCursor = (missingCursor + 25) % Math.max(1, totalGrid);
  if (missingIds.length === 0) return;

  isFetchingTiles = true;
  try {
    const ctrl = new AbortController();
    const tid = setTimeout(() => ctrl.abort(), 1600);
    const r = await fetch(`/api/tiles?ids=${missingIds.join(',')}`, {signal: ctrl.signal});
    clearTimeout(tid);
    const d = await r.json();
    if (d.w && d.h) {
      applyCanvasResolution(d.w, d.h);
    }
    if (d.hdr && d.tiles && d.tiles.length > 0) {
      await drawTileBatch(d.hdr, d.tiles);
    }
  } catch (e) {
  } finally {
    isFetchingTiles = false;
  }
}

async function pollLoop() {
  try {
    const ctrl = new AbortController();
    const tid = setTimeout(() => ctrl.abort(), 1600);
    const r = await fetch('/api/status', {signal: ctrl.signal});
    clearTimeout(tid);
    const d = await r.json();
    latestStatus = d;

    if (d.imgW && d.imgH) {
      applyCanvasResolution(d.imgW, d.imgH);
    }

    if (lastClearSeq !== -1 && d.clearSeq !== lastClearSeq) {
      lastTileSeq = 0;
      lastImgVer = 0;
      lastCaptureId = 0;
      resetDrawnTiles();
      drawBlankCanvas('Screen Cleared · Standby');
      document.getElementById('progBar').style.width = '0%';
    }
    lastClearSeq = d.clearSeq;

    if (lastCaptureId !== -1 && d.captureId !== lastCaptureId && d.captureId > 0) {
      lastTileSeq = 0;
      resetDrawnTiles();
    }
    lastCaptureId = d.captureId;

    document.getElementById('lvlSel').value = d.level;
    document.getElementById('ageSpan').textContent = d.totalPkts > 0
      ? `${d.rssi.toFixed(0)} dBm · SNR ${d.snr >= 0 ? '+' : ''}${d.snr.toFixed(1)} dB (${d.ageSec}s ago)`
      : 'Waiting for signal';

    const linkDot = document.getElementById('linkDot');
    const linkTitle = document.getElementById('linkTitle');
    if (d.dropped) {
      linkDot.className = 'dot offline';
      linkTitle.textContent = `Signal Standby (${d.ageSec}s)`;
    } else {
      linkDot.className = 'dot';
      linkTitle.textContent = `Drone Connected · 868 MHz`;
    }

    updateSignalIndicators(d.rssi, d.snr, d.totalPkts, d.dropped);

    const pct = d.totalChunks > 0 ? Math.round((d.receivedChunks / d.totalChunks) * 100) : 0;
    document.getElementById('progBar').style.width = pct + '%';

    const unitLbl = (d.mode === '2D_FULL') ? 'Packets' : 'Blocks';
    document.getElementById('hudBlocks').innerHTML = (d.totalChunks > 0 && d.receivedChunks > 0)
      ? `${curImgW}×${curImgH} · ${unitLbl}: <b>${d.receivedChunks} / ${d.totalChunks} (${pct}%)</b>`
      : `${curImgW}×${curImgH} Viewport · <b>Standby</b>`;
    document.getElementById('hudTarget').innerHTML = (d.captureId > 0 && d.maxConf > 0)
      ? `Capture #${d.captureId} · <b>Target ${d.maxConf.toFixed(0)}%</b>`
      : (d.liveConf > 0 ? `Live Tracking: <b>${d.liveConf.toFixed(0)}%</b>` : `Optical Feed · <b>Standby</b>`);
    document.getElementById('hudGps').innerHTML = `GPS: <b>${d.gps || 'No Fix'}</b>`;
    document.getElementById('hudMode').innerHTML = `Mode: <b>${formatModeLabel(d.mode)}</b>`;

    document.getElementById('capTimeBadge').textContent = d.timeStr || '--:--:--';
    document.getElementById('valState').textContent = d.droneState || 'WIDE_SEARCH';
    document.getElementById('valModel').textContent = `${d.model || 'SSD'} · Q${d.jpgQuality}`;
    document.getElementById('valNote').textContent = d.note || '--';
    document.getElementById('valDiag').textContent = `${d.totalPkts} pkts · ${d.corrupted} CRC · ${d.flashKb} KB`;

    drawRssiChart(d.rssiHist, d.dropped);

    if (d.totalPkts > 0) {
      const ts = new Date().toLocaleTimeString();
      if (rssiLogRows.length === 0 || rssiLogRows[rssiLogRows.length - 1].pkt !== d.totalPkts) {
        rssiLogRows.push({
          time: ts, pkt: d.totalPkts, rssi: d.rssi, snr: d.snr,
          dropped: d.dropped ? 'DROPPED' : 'OK', state: d.droneState,
          conf: d.maxConf, gps: d.gps, mode: formatModeLabel(d.mode)
        });
        if (rssiLogRows.length > 500) rssiLogRows.shift();
      }
    }

    // 1. Stream newly arrived blocks
    if (d.tileSeq > lastTileSeq) {
      await fetchPictureBlocks(lastTileSeq);
      checkAutoLogCapture(d, pct);
    }
    // 2. Actively reconcile any missed blocks by requesting their exact grid IDs
    if (d.mode !== '2D_FULL' && d.captureId > 0 && d.totalChunks > 0 && d.receivedChunks > 0 && d.hdr && drawnCount < (curTileCols * curTileRows)) {
      await reconcileMissingBlocks();
      checkAutoLogCapture(d, pct);
    }
  } catch (e) {}
  setTimeout(pollLoop, 500);
}

function checkAutoLogCapture(d, pct) {
  if (!document.getElementById('autoSaveChk').checked) return;
  if (d.captureId > 0 && pct >= 90 && lastAutoLoggedCaptureId !== d.captureId) {
    lastAutoLoggedCaptureId = d.captureId;
    addCaptureToLocalHistory(d);
  }
}

function addCaptureToLocalHistory(d) {
  const dataUrl = vCanvas.toDataURL('image/jpeg', 0.90);
  const rec = {
    id: d.captureId,
    time: d.timeStr || new Date().toLocaleTimeString(),
    conf: d.maxConf.toFixed(0),
    mode: formatModeLabel(d.mode),
    gps: d.gps,
    note: d.note,
    rssi: d.rssi.toFixed(0),
    snr: d.snr.toFixed(1),
    imgData: dataUrl
  };
  savedCaptures.unshift(rec);
  if (savedCaptures.length > 12) savedCaptures.pop();
  try { localStorage.setItem('sar_saved_captures', JSON.stringify(savedCaptures)); } catch (e) {}
  renderSavedHistoryList();
}

function renderSavedHistoryList() {
  const box = document.getElementById('savedList');
  if (savedCaptures.length === 0) {
    box.innerHTML = '<div style="color:#6b7280;padding:4px 0;">No captures saved yet.</div>';
    return;
  }
  box.innerHTML = savedCaptures.map((c, idx) =>
    `<div class="log-row">
      <span><b>#${c.id}</b> · ${c.time} · <b>${c.conf}%</b> · ${c.gps} · ${c.rssi} dBm</span>
      <button onclick="downloadHistoryItem(${idx})" style="padding:4px 10px;font-size:10px;">Save</button>
    </div>`
  ).join('');
}

function downloadHistoryItem(idx) {
  const c = savedCaptures[idx];
  if (!c) return;
  const a = document.createElement('a');
  a.href = c.imgData;
  a.download = `SAR_Capture_${c.id}_${c.conf}pct.jpg`;
  a.click();
}

function saveCanvasJpg() {
  const cid = latestStatus.captureId || 0;
  const conf = latestStatus.maxConf ? latestStatus.maxConf.toFixed(0) : 0;
  const a = document.createElement('a');
  a.href = vCanvas.toDataURL('image/jpeg', 0.92);
  a.download = `SAR_Capture_${cid}_${conf}pct_${curImgW}x${curImgH}.jpg`;
  a.click();
}

function saveFullReportHtml() {
  const d = latestStatus;
  const imgData = vCanvas.toDataURL('image/jpeg', 0.92);
  const rowsHtml = rssiLogRows.slice(-60).map(r =>
    `<tr><td>${r.time}</td><td>${r.pkt}</td><td>${r.rssi.toFixed(1)} dBm</td><td>${r.snr.toFixed(1)} dB</td><td>${r.dropped}</td><td>${r.conf}%</td><td>${r.gps}</td></tr>`
  ).join('');
  const html = `<!DOCTYPE html><html><head><meta charset="utf-8"><title>SAR Report #${d.captureId || 0}</title>
  <style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#111317;color:#f3f4f6;padding:24px;}img{max-width:100%;border:1px solid #2a2f3a;border-radius:8px;}table{border-collapse:collapse;width:100%;margin-top:14px;}td,th{border:1px solid #2a2f3a;padding:7px;text-align:left;font-size:12px;}</style></head>
  <body><h2>SAR Ground Report · Capture #${d.captureId || 0} (${curImgW}×${curImgH})</h2>
  <p><b>Time:</b> ${d.timeStr || new Date().toLocaleString()} | <b>Target Confidence:</b> ${(d.maxConf||0).toFixed(1)}% | <b>Mode:</b> ${formatModeLabel(d.mode)} | <b>Detector:</b> ${d.model||''}</p>
  <p><b>GPS Coordinates:</b> ${d.gps||''} | <b>Unit:</b> ${d.note||''}</p>
  <p><b>Signal Power:</b> ${d.rssi||0} dBm / SNR ${d.snr||0} dB (Min ${d.minRssi||0} dBm, Max ${d.maxRssi||0} dBm)</p>
  <img src="${imgData}" alt="SAR Capture">
  <h3>RF Signal History</h3>
  <table><tr><th>Time</th><th>Packet</th><th>RSSI (dBm)</th><th>SNR (dB)</th><th>Link</th><th>Confidence</th><th>GPS</th></tr>${rowsHtml}</table>
  </body></html>`;
  const blob = new Blob([html], {type: 'text/html'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `SAR_Report_${d.captureId || 0}.html`;
  a.click();
}

function saveRssiCsvLog() {
  let csv = "Time,PacketNum,RSSI_dBm,SNR_dB,LinkStatus,DroneState,MaxConf_Pct,Mode,GPS\n";
  for (const r of rssiLogRows) {
    csv += `"${r.time}",${r.pkt},${r.rssi},${r.snr},${r.dropped},"${r.state}",${r.conf},"${r.mode}","${r.gps}"\n`;
  }
  const blob = new Blob([csv], {type: 'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `SAR_Signal_Log.csv`;
  a.click();
}

async function changeLevel(lvl) {
  await fetch('/api/level?set=' + lvl);
}

async function changeResolution(val) {
  const parts = val.split('x');
  if (parts.length === 2) {
    const w = parseInt(parts[0], 10);
    const h = parseInt(parts[1], 10);
    applyCanvasResolution(w, h, `Manual Viewport: ${w}×${h}`);
    try {
      await fetch(`/api/res?w=${w}&h=${h}`);
    } catch (e) {}
  }
}

try {
  const raw = localStorage.getItem('sar_saved_captures');
  if (raw) savedCaptures = JSON.parse(raw);
} catch (e) {}
renderSavedHistoryList();
pollLoop();
</script>
</body>
</html>
"""

# Sender socket to send commands back to demo sender
sender_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class SARReceiverHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence default HTTP access logs for clean console
        return

    def do_GET(self):
        global current_level
        url_parsed = urllib.parse.urlparse(self.path)
        path = url_parsed.path
        query = urllib.parse.parse_qs(url_parsed.query)

        # 1. Main Dashboard
        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))
            return

        # 2. Status API
        if path == "/api/status":
            with state_lock:
                now = time.time()
                age_sec = int(now - last_packet_time) if last_packet_time > 0 else 999
                dropped = (last_packet_time == 0) or (age_sec >= 7)
                flash_kb = 48  # Simulated LittleFS usage

                hist_json = json.dumps([int(r) for r in rssi_history])

                status_dict = {
                    "captureId": det_capture_id,
                    "personCount": det_person_count,
                    "maxConf": round(det_max_conf, 1),
                    "liveConf": round(drone_live_conf, 1),
                    "hbSeq": heartbeat_seq,
                    "droneState": drone_zoom_state,
                    "mode": det_mode_str,
                    "model": det_model_str,
                    "timeStr": det_time_str,
                    "gps": det_gps_str,
                    "note": det_note_str,
                    "jpgQuality": det_jpg_quality,
                    "imgW": img_width,
                    "imgH": img_height,
                    "tileCols": tile_cols,
                    "tileRows": tile_rows,
                    "receivedChunks": received_chunks,
                    "totalChunks": total_chunks,
                    "corrupted": corrupted_packets,
                    "rssi": round(last_rssi, 1),
                    "snr": round(last_snr, 1),
                    "minRssi": round(min_rssi, 1),
                    "maxRssi": round(max_rssi, 1),
                    "totalPkts": total_packets_rx,
                    "drops": link_drop_count,
                    "dropped": dropped,
                    "flashKb": flash_kb,
                    "rssiHist": [int(r) for r in rssi_history],
                    "imgVer": image_version,
                    "tileSeq": global_tile_seq,
                    "clearSeq": clear_seq,
                    "level": current_level,
                    "levelName": LEVEL_NAMES[current_level] if current_level < len(LEVEL_NAMES) else f"L{current_level}",
                    "ageSec": age_sec
                }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(status_dict).encode("utf-8"))
            return

        # 3. Tiles API
        if path == "/api/tiles":
            with state_lock:
                tiles_to_send = []
                has_more = False
                next_from = active_num_tiles

                if "ids" in query:
                    # Specific block IDs requested by the phone for missing-block reconciliation
                    id_strings = query["ids"][0].split(",")
                    count = 0
                    for s in id_strings:
                        if not s.strip().isdigit():
                            continue
                        t_id = int(s.strip())
                        if 0 <= t_id < active_num_tiles and t_id < MAX_TILES:
                            slot = tile_slots[t_id]
                            if slot:
                                r = t_id // tile_cols
                                c = t_id % tile_cols
                                tiles_to_send.append({
                                    "c": c, "r": r,
                                    "seq": tile_seqs[t_id],
                                    "b64": slot
                                })
                                count += 1
                                if count >= 25:
                                    break
                else:
                    since_seq = int(query["since"][0]) if "since" in query else 0
                    from_idx = int(query["from"][0]) if "from" in query else 0
                    count = 0
                    for t in range(from_idx, min(active_num_tiles, MAX_TILES)):
                        slot = tile_slots[t]
                        if slot and tile_seqs[t] > since_seq:
                            if count >= 20:
                                has_more = True
                                next_from = t
                                break
                            r = t // tile_cols
                            c = t % tile_cols
                            tiles_to_send.append({
                                "c": c, "r": r,
                                "seq": tile_seqs[t],
                                "b64": slot
                            })
                            count += 1

                resp_obj = {
                    "w": img_width,
                    "h": img_height,
                    "hdr": common_hdr_b64,
                    "maxSeq": global_tile_seq,
                    "tiles": tiles_to_send,
                    "more": has_more,
                    "nextFrom": next_from
                }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(resp_obj).encode("utf-8"))
            return

        # 4. Clear Screen API
        if path == "/api/clear":
            clear_receiver_state()
            try:
                sender_udp_socket.sendto(b"CMD:CLEAR\n", ("127.0.0.1", DEFAULT_SENDER_UDP_PORT))
            except Exception:
                pass
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"CLEARED")
            return

        # 5. Level Set API
        if path == "/api/level":
            if "set" in query:
                try:
                    lvl = int(query["set"][0])
                    current_level = lvl
                    # Forward CMD:LEVEL:<lvl> to sender
                    cmd = f"CMD:LEVEL:{lvl}\n".encode("utf-8")
                    sender_udp_socket.sendto(cmd, ("127.0.0.1", DEFAULT_SENDER_UDP_PORT))
                except Exception:
                    pass
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK")
            return

        # 6. Resolution Set API
        if path == "/api/res":
            if "w" in query and "h" in query:
                try:
                    w = int(query["w"][0])
                    h = int(query["h"][0])
                    set_resolution_grid(w, h)
                    # Forward resolution change to sender
                    cmd = f"CMD:RES:{w}:{h}\n".encode("utf-8")
                    sender_udp_socket.sendto(cmd, ("127.0.0.1", DEFAULT_SENDER_UDP_PORT))
                except Exception:
                    pass
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK")
            return

        # Fallback 404
        self.send_response(404)
        self.end_headers()

def lora_udp_listener(port=DEFAULT_LORA_UDP_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
    except Exception as e:
        print(f"[ERROR] Receiver UDP Listener failed to bind port {port}: {e}")
        return
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            line = data.decode("ascii", errors="ignore")
            # Virtual RF simulation headers can be passed as prefix e.g. [RSSI=-65;SNR=9.2]LINE
            rssi_sim = None
            snr_sim = None
            if line.startswith("[RSSI="):
                end_sq = line.find("]")
                if end_sq > 0:
                    meta = line[1:end_sq]
                    line = line[end_sq + 1:]
                    for part in meta.split(";"):
                        if part.startswith("RSSI="):
                            rssi_sim = float(part[5:])
                        elif part.startswith("SNR="):
                            snr_sim = float(part[4:])
            handle_lora_packet(line, rssi_sim, snr_sim)
        except Exception:
            pass

def start_server(http_port=DEFAULT_HTTP_PORT, lora_port=DEFAULT_LORA_UDP_PORT):
    # Start background LoRa UDP listener
    t_udp = threading.Thread(target=lora_udp_listener, args=(lora_port,), daemon=True)
    t_udp.start()

    # Find free HTTP port
    actual_port = http_port
    server = None
    for p in range(http_port, http_port + 20):
        try:
            server = HTTPServer(("0.0.0.0", p), SARReceiverHandler)
            actual_port = p
            break
        except OSError:
            continue

    if not server:
        print(f"[ERROR] Could not bind HTTP server to any port near {http_port}!")
        return None, None

    local_ip = get_local_ip()
    print("=" * 76)
    print(" SAR DRONE SYSTEM — VIRTUAL GROUND RECEIVER & MOBILE WEB SERVER")
    print("=" * 76)
    print(f" [OK] Virtual 868 MHz LoRa Radio listening on UDP port {lora_port}")
    print(f" [OK] Ground Web Dashboard is LIVE:")
    print(f"      -> Local Browser:  http://localhost:{actual_port}/")
    print(f"      -> Mobile / Phone: http://{local_ip}:{actual_port}/")
    print("=" * 76)
    print(" Rescuers with smartphones on the same Wi-Fi can open the Mobile URL!")
    print(" Starting web server thread...\n")

    t_http = threading.Thread(target=server.serve_forever, daemon=True)
    t_http.start()
    return server, actual_port

if __name__ == "__main__":
    server, port = start_server()
    if server:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down receiver server.")
