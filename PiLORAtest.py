import serial
import time
import sys

try:
    # Connect to the Pi's hardware serial port at 9600 baud
    lora = serial.Serial('/dev/serial0', 9600, timeout=1)
    print("LoRa port opened successfully on /dev/serial0")
except Exception as e:
    print(f"Failed to open port: {e}")
    sys.exit()

print("Starting automated test. Sending 'hello' every 5 seconds...")
print("Press the red 'Stop' button in Thonny to end.")

try:
    while True:
        # Create the message with a newline character so the ESP32 knows when it ends
        message = "hello\n" 
        
        # Send it to the LoRa HAT
        lora.write(message.encode('utf-8'))
        print(f"[Sent to ESP32]: hello")
        
        # Wait exactly 5 seconds before looping
        time.sleep(5)
        
except KeyboardInterrupt:
    print("\nClosing connection...")
    lora.close()
