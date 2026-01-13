'''
-----------------------------------------------------
-----------------------------------------------------
Lunabotics Motor Testing
Author: NotAWildernessExplorer
Date:02/12/2025 


-----------------------------------------------------
-----------------------------------------------------
Use MOSFET LVL shifter for UART_TX so the Sabertooth 
can read us

Make Sure Sabertooth's dip switches are set to:

	#1 down
	#2 down
	#3 up
	#4 up
	#5 up
	#6 up

-----------------------------------------------------
-----------------------------------------------------
If access is denied to the serial port, run command
$sudo chmod 666 /dev/serial0

'''

## import library 
import time				# What time is it? Well, this library will tell you!!! 
from pysabertooth import Sabertooth


## Init up the sabertooth 1, and open the seral connection 
motor1 = Sabertooth("/dev/serial0", baudrate = 9600, address = 128)	# Init the Motor
motor1.open()								# Open then connection
print(f"Connection Status: {motor1.saber.is_open}")			# Let us know if it is open
motor1.info()								# Get the motor info


## Init up the sabertooth 2, and open the seral connection 
motor2 = Sabertooth("/dev/serial0", baudrate = 9600, address = 129)	# Init the Motor
motor2.open()								# Open then connection
print(f"Connection Status: {motor2.saber.is_open}")			# Let us know if it is open
motor2.info()								# Get the motor info


## Turn the Motors one at a time
## DO NOT GO ABOVE 70% IT WILL BLOW A FUSE!!!
power_lvl = 80			# Int [-100,100] where < 0  is reverse

## Motor 1
motor1.drive(1,power_lvl)	# Turn on motor 1
print("i should be spinning now.")
motor1.drive(2,power_lvl)	# Turn on motor 2

time.sleep(0.5)

power_lvl:int = -80			# Int [-100,100] where < 0  is reverse
## Motor 2
motor2.drive(1,power_lvl)	# Turn on motor 1
motor2.drive(2,power_lvl)	# Turn on motor 2
time.sleep(20)			# Pause for a few seconds sec

motor1.stop()			# Turn off both motors
motor2.stop()			# Turn off both motors



## Delete the connection and shut down
print("cya nerds")
del motor1
del motor2
