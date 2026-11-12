# Search & Rescue (SAR) Drone AI System — Full Hardware-Free Demo

This demo allows **anyone to experience the complete Search and Rescue Drone AI camera and LoRa transmission system without needing any physical hardware** (no Raspberry Pi, no Sony IMX500 AI Camera, and no ESP32-S3 LoRa boards required!).

It simulates the entire airborne drone mission control, on-sensor neural network target detection, tracking scope gimbal, multi-resolution opposite-corner PiP composer, 868 MHz LoRa physical layer, and ground station mobile web dashboard over a local loopback link.

---

## ⚡ Quick Start (1-Click)

### Windows
Download whole demo folder
Double-click **`Start_Demo.bat`** in this folder (or in the project root).

### Windows / macOS / Linux (Terminal)
```bash
# 1. Install Pillow (only dependency required!)
pip install pillow

# 2. Run the 1-click master launcher
python run_demo.py
```

This single command will:
1. Launch the **Virtual Ground Station & Mobile Web Server** (`demo_ground_receiver.py`).
2. Automatically pop open your web browser to the **Ground Rescuer Mobile HUD** (`http://localhost:8088/`).
3. Open the **Airborne Drone Mission Control GUI** (`demo_drone_sender.py`).

---

## 📱 Connecting a Real Smartphone (Optional)

Because the virtual ground receiver runs a real HTTP server on port `8088`, you can pull out your phone, make sure it's on the same home or office Wi-Fi as your computer, and open:
```text
http://<YOUR-COMPUTER-LAN-IP>:8088/
```
*(The receiver console prints your exact LAN IP address upon startup!)*

You will see the live blocks stream onto your real phone screen, block by block, just like a field rescuer in wilderness operations!

---

## 🎮 What to Try in the Demo

### 1. Interactive SAR Search & Target Tracking
- In the **Wide Optical Sensor** viewport (top left), click anywhere on the terrain to move the lost person.
- Watch the **Target Tracking Scope** (bottom left) smoothly swing its digital gimbal and zoom in ($2.5\times$).
- Notice the **2-Stage Lock State Machine**:
  - `WIDE_SEARCH` $\rightarrow$ counts frames $\ge 38\%$ trigger.
  - `ZOOM_CONFIRM` $\rightarrow$ verifies candidate $\ge 58\%$ over 3 consecutive frames.
  - `TARGET LOCKED` $\rightarrow$ target confirmed! Click **Transmit Now** to broadcast the snapshot immediately, or check **Auto-transmit** to stream automatically whenever a target is locked!

### 2. Manual & Automatic Transmission
- **Manual Mode (Default):** The simulator tracks and locks onto the lost hiker in standby. Hit **Transmit Now** whenever you want to send a fresh tactical snapshot to ground rescuers.
- **Auto-Transmit Mode:** Check the *"Auto-transmit when a target is confirmed"* box to automatically trigger snapshot broadcasts whenever the 2-stage lock confirms a target.
- **Clear Screen & Cancel:** Click **Clear Phone Screen** on Mission Control (or **Clear Screen** on the mobile phone HUD) at any time to instantly stop an active transmission and wipe the receiver viewport back to clean standby!

### 2. Auto-Opposite Picture-in-Picture (PiP) Inset
- Move the target to different quadrants (Top-Left, Top-Right, Bottom-Left, Bottom-Right).
- When a broadcast triggers (or when you click **Transmit Now**), look at your browser HUD: the zoomed PiP inset is placed in the **diagonally opposite corner**, guaranteeing the detected person is never obscured!

### 3. Target-First Radial Block Ordering
- Notice how the picture materializes on the ground receiver: the blocks containing the detected person appear **first**, while the surrounding wilderness fills in radially outward.

### 4. Simulating Real-World RF Packet Loss & Rescan Repair
- Adjust the **Packet Loss Rate (%)** slider to `15%` or `20%`.
- Hit **Transmit Now**:
  - In Pass 1, several blocks will be dropped due to simulated interference, leaving visible gaps in the canvas.
  - In Pass 2 (Rescan), the system actively queries the missing block IDs (`/api/tiles?ids=...`) and repairs every single dropped hole until the image is 100% complete!

### 5. Multi-Resolution Selection
- Switch resolutions on the fly:
  - `320×240` (75 blocks — ultra-fast preview)
  - `640×480` (300 blocks — standard SAR resolution)
  - `960×720` (675 blocks — high detail)
  - `1280×960` (1200 blocks — maximum optical clarity)
- Notice that the mobile phone viewport automatically resizes its canvas and redraws the active grid immediately!

### 6. Exporting Field Reports
- Click **Save Photo** to export a timestamped high-res JPEG.
- Click **Export Report** to generate a complete standalone HTML Search & Rescue incident report with embedded imagery, coordinates, confidence, and RF signal history.
- Click **Signal CSV** to download a spreadsheet of signal power (`dBm`), `SNR`, and link status over time.

---

## 🧩 Architecture & File Structure

```text
demo/
├── run_demo.py               # Master 1-click launcher (starts both sender & receiver)
├── Start_Demo.bat            # Windows 1-click shortcut
├── demo_drone_sender.py      # Airborne Drone Simulator & Mission Control GUI (Tkinter + PIL)
├── demo_ground_receiver.py   # Ground Station & Mobile Web Server (pure standard library Python)
├── generate_sample_scene.py  # Procedural SAR wilderness terrain generator
├── sample_sar_aerial.jpg     # 2028x1520 high-res SAR aerial test scene
└── README.md                 # This documentation
```

### Virtual RF Link (Protocol Compatibility)
The demo scripts use the **exact same byte-for-byte ASCII Over-The-Air wire protocol** as the physical hardware:
- `CFG:` Instant configuration sync
- `H:` Heartbeat & live telemetry (2.5s)
- `D:` Target alert & resolution header
- `M:` GPS, timestamp, and sector dispatch note
- `JH:` Shared 64×16 JPEG quantization header
- `J5:` / `J10:` / `J15:` / `J20:` 64×16 24-bit JPEG picture block payloads
- `F:` Full-frame progressive JPEG slices
- `CMD:LEVEL:` / `CMD:CLEAR` Remote ground station control

Packets are routed over local UDP socket port `9868` (downlink) and port `9869` (uplink), allowing the demo to run on a single machine or between two separate computers over local Wi-Fi.
