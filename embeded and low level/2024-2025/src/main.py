import time
import Actuator 
import LightSabers 
import cancancan
from sshkeyboard import listen_keyboard
from pysabertooth import Sabertooth
import sys
import RPi.GPIO as GPIO

## dictionary that holds our telemetry
telemetry_data = {}


# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
# -----------------------Helper Functions--------------------------------
# -----------------------------------------------------------------------
# -----------------------------------------------------------------------

# Key release function
def release(key):
    print("\rPress a key!                    ",end = "")
    print("\rPress a key!                    ",end = "")
    Lynn.stop()
    Rego.stop()
    Derive.stop()
    # telemetry_data = cancancan.read_can(telemetry_data)
# Key press function
def press(key):
    lin_speed:int  = 70
    turn_speed:int = 50
    dig_speed:int  = 100

    print(f"\rYou pressed {key}!",end = "")
    
    
    key = key.lower()
    if key == 'p':
        GPIO.cleanup()
        Lynn.stop()
        Rego.stop()
        Derive.stop()
        sys.exit(0)
    if  key == 'w':    
        Derive.linear_motion(-lin_speed)
    elif key == 's':
        Derive.linear_motion(lin_speed)
    elif key == 'a':
        Derive.turn_motion(-turn_speed)
    elif key == 'd':
    	Derive.turn_motion(turn_speed)
    elif key == "e":
        Lynn.move(+1)
    elif key == "q":
        Lynn.move(-1)
    elif key == "space":
        Rego.dig(dig_speed)
    elif key == "r":
        Rego.deposition()
    else: 
        print("\rN3RD!                  ",end = "") 

# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
# --------------------------Entering Main--------------------------------
# -----------------------------------------------------------------------
# -----------------------------------------------------------------------


if __name__ == '__main__':
    Lynn    = Actuator.linearactuator()
    Derive  = LightSabers.DriveTrain()
    Rego    = LightSabers.CheeseGrater()

    print("-"*15)
    print("-"*15)
    print("Starting The Fun!")
    
    time.sleep(0.05)
    while True:
            listen_keyboard(on_press=press, on_release=release, sequential=True)
            time.sleep(0.001)
