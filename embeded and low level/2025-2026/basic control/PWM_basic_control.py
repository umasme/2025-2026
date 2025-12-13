#Auther: Ahmed Ahmed
# Date: 8/22/2025
# Organization: ASME
# Purpose: This code is simply to control the PWM using gpiozero lib. 
# Other libraries were tried and did not work on pi5


from gpiozero import PWMLED
from time import sleep
import math

# Pick your pin here:
# GPIO12 or GPIO18 → PWM0
# GPIO13 or GPIO19 → PWM1
# gpiozero uses the hardware PWM when you choose these GPIOs. 
led = PWMLED(12)  

print("Fading LED... CTRL+C to stop.")

try:
    while True:
        for i in range(360):
            brightness = (math.sin(math.radians(i)) + 1) / 2
            led.value = brightness  # value must be 0.0–1.0 crossponds to 0% to 100% dutycycle
            sleep(0.01)
except KeyboardInterrupt:
    print("\nExiting.")
