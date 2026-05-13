import can
import time
import struct

# Initialize CAN0 interface
try:
    # Ensure bitrate matches Teensy 4.1 (1,000,000)
    bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
    print("Jetson CAN interface ready. Press Ctrl+C to stop.")
except Exception as e:
    print(f"Interface Error: {e}")
    exit()

def send_rover(m1, m2, m3):
    """
    Packs data for Teensy:
    - Addr 129 (Unsigned) + M1 Speed (Signed)
    - Addr 130 (Unsigned) + M2 Speed (Signed) + M3 Speed (Signed)
    """
    # Clamp to Sabertooth safety limits (-127 to 127)
    m1, m2, m3 = [max(-127, min(127, s)) for s in [m1, m2, m3]]

    # '>BbBbb' -> Big-endian: Unsigned, Signed, Unsigned, Signed, Signed
    data = struct.pack('>BbBbb', 129, m1, 130, m2, m3)
    data += b'\x00' * (8 - len(data)) # Pad to 8 bytes
   
    msg = can.Message(arbitration_id=0x100001, data=data, is_extended_id=True)
   
    try:
        bus.send(msg)
    except can.CanError:
        print("CAN Transmission Error")

def continuous_sweep():
    cycle_count = 1
    try:
        while True:
            print(f"--- Starting Cycle {cycle_count} ---")
           
            # 1. Sweep Forward (0 to 100)
            print("Action: Sweeping Forward")
            for s in range(0, 101, 10):
                send_rover(s, s, s)
                time.sleep(0.1)
               
            # 2. Sweep back to Reverse (100 to -100)
            print("Action: Reversing Direction")
            for s in range(100, -101, -10):
                send_rover(s, s, s)
                time.sleep(0.1)
               
            # 3. Return to Stop (-100 to 0)
            print("Action: Returning to Neutral")
            for s in range(-100, 1, 10):
                send_rover(s, s, s)
                time.sleep(0.1)

            # Brief pause at zero before next cycle
            send_rover(0, 0, 0)
            print(f"Cycle {cycle_count} Complete. Pausing...\n")
            time.sleep(1.0)
           
            cycle_count += 1

    except KeyboardInterrupt:
        # Emergency stop on exit
        print("\nStopping script... Sending zero-speed command.")
        for _ in range(5): # Send multiple times to ensure reception
            send_rover(0, 0, 0)
            time.sleep(0.05)
        print("Rover Halted.")

if __name__ == "__main__":
    continuous_sweep()
