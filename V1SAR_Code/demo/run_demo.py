#!/usr/bin/env python3
"""
========================================================================================
Search & Rescue (SAR) Drone System — 1-Click Master Demo Launcher
========================================================================================
Launches both:
  1. Virtual Ground Station & Phone Web Server (demo_ground_receiver.py)
  2. Airborne Drone AI Camera & LoRa Transmitter GUI (demo_drone_sender.py)
  3. Automatically opens your default web browser to the Ground Station Phone HUD!
========================================================================================
"""

import sys
import os
import time
import subprocess
import webbrowser

def main():
    demo_dir = os.path.dirname(os.path.abspath(__file__))
    receiver_script = os.path.join(demo_dir, "demo_ground_receiver.py")
    sender_script = os.path.join(demo_dir, "demo_drone_sender.py")

    # 1. Verify Pillow is installed
    try:
        import PIL
    except ImportError:
        print("=" * 72)
        print(" [!] Missing Required Dependency: Pillow")
        print("     Please install Pillow by running:  pip install pillow")
        print("=" * 72)
        input("Press Enter to exit...")
        sys.exit(1)

    print("=" * 72)
    print(" STARTING SAR DRONE SYSTEM FULL HARDWARE-FREE DEMO")
    print("=" * 72)
    print(" 1. Launching Virtual Ground Receiver & Web Server...")

    # Start the receiver process
    receiver_proc = subprocess.Popen([sys.executable, receiver_script], cwd=demo_dir)
    time.sleep(1.2)

    # Automatically open web browser to the Ground Station HUD
    # Try port 8088 (default)
    print(" 2. Opening Ground Rescuer Web Dashboard in your browser...")
    webbrowser.open("http://localhost:8088/")

    print(" 3. Launching Airborne Drone AI Mission Control GUI...")
    print("=" * 72)
    print(" Instructions:")
    print("   • Click anywhere on the Wide Optical Sensor to move the lost person.")
    print("   • Watch the 2-stage tracking scope zoom in and confirm the lock.")
    print("   • Hit 'Transmit Now' to broadcast blocks over the virtual LoRa link.")
    print("   • Watch blocks stream into your browser HUD in Target-First order!")
    print("   • Use the 'Packet Loss' slider to test RF interference & Pass 2 repair!")
    print("=" * 72)

    try:
        # Run sender in foreground (blocks until closed)
        subprocess.run([sys.executable, sender_script], cwd=demo_dir)
    finally:
        print("\nShutting down demo processes...")
        try:
            receiver_proc.terminate()
            receiver_proc.wait(timeout=2)
        except Exception:
            pass
        print("Demo completed.")

if __name__ == "__main__":
    main()
