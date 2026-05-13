import can
import struct
import time

# --- P-Controller Configuration ---
KP_ACTUATOR = 0.8    
KP_DEPOSITION = 2.8  # Aggressive gain to fix slowness
DEPO_MIN_SPEED = 20  # Minimum power floor to overcome stiction
ACT_MIN_SPEED = 12

# --- CALIBRATION & TARGETS ---
# Relative distance to move deposition (the "Difference")
DEPO_RUN_DISTANCE_CM = 35 * 2.45  

# Absolute height for the actuators
TARGET_ACT_CM = 5.5  

# --- Global State ---
current_adc = 0
current_cm = 0.0
target_depo_absolute = None  # Will be set based on initial position

# Initialize Bus
try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
except Exception as e:
    print(f"CAN Init Error: {e}")
    exit(1)

def cm_to_adc(cm):
    """Calibrated formula: ADC = (42.73 * cm) + 20"""
    return int((42.73 * cm) + 20)

def drain_can_bus():
    """Drains buffer to get the absolute freshest telemetry."""
    global current_adc, current_cm
    while True:
        msg = bus.recv(timeout=0.0)
        if msg is None: break
        if msg.arbitration_id == 0x200001:
            # Unpack Actuator ADC (2 bytes) and Depo Float (4 bytes)
            current_adc = (msg.data[1] << 8) | msg.data[0]
            current_cm = struct.unpack('<f', msg.data[2:6])[0]

def run_p_controller():
    global target_depo_absolute
    drain_can_bus()

    # --- INITIALIZATION STEP ---
    # On the first loop, capture current position and set the relative target
    if target_depo_absolute is None:
        if current_cm == 0.0 and current_adc == 0:
            # Wait for first valid telemetry before setting target
            return
        target_depo_absolute = current_cm + DEPO_RUN_DISTANCE_CM
        print(f"Start: {current_cm:.1f} cm | Target: {target_depo_absolute:.1f} cm")

    # --- 1. Linked Actuators (Address 130) ---
    target_adc = cm_to_adc(TARGET_ACT_CM)
    error_act = target_adc - current_adc
    if abs(error_act) < 5:
        act_speed = 0
    else:
        act_speed = int(error_act * KP_ACTUATOR)
        # Apply Power Floor
        if act_speed > 0: act_speed += ACT_MIN_SPEED
        elif act_speed < 0: act_speed -= ACT_MIN_SPEED

    # --- 2. Deposition (Address 129) - Relative Logic ---
    error_depo = target_depo_absolute - current_cm
    if abs(error_depo) < 0.2: # Deadband 2mm
        depo_speed = 0
    else:
        depo_speed = int(error_depo * KP_DEPOSITION)
        # Apply Power Floor
        if depo_speed > 0: depo_speed += DEPO_MIN_SPEED
        elif depo_speed < 0: depo_speed -= DEPO_MIN_SPEED

    # --- 3. Safety Clamp & Transmission ---
    act_speed = max(min(act_speed, 127), -127)
    depo_speed = max(min(depo_speed, 127), -127)

    payload = struct.pack('<hh', act_speed, depo_speed)
    bus.send(can.Message(arbitration_id=0x100001, data=payload, is_extended_id=True))

    print(f"ACT: {current_adc:4d}/{target_adc} | DEPO: {current_cm:5.1f}/{target_depo_absolute:5.1f} | Spd: {depo_speed:3d}")

if __name__ == "__main__":
    print(">>> RELATIVE DISTANCE CONTROLLER ACTIVE <<<")
    try:
        while True:
            run_p_controller()
            time.sleep(0.05) # 20Hz Loop
    except KeyboardInterrupt:
        bus.send(can.Message(arbitration_id=0x100001, data=struct.pack('<hh', 0, 0), is_extended_id=True))
        bus.shutdown()