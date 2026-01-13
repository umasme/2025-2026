import linearactuator
import time
from pysabertooth import Sabertooth

motor1 = Sabertooth("/dev/serial0", baudrate = 9600, address = 129)	# Init the Motor
motor1.open()								# Open then connection
print(f"Connection Status: {motor1.saber.is_open}")			# Let us know if it is open
motor1.info()								# Get the motor info


## Init up the sabertooth 2, and open the seral connection 
motor2 = Sabertooth("/dev/serial0", baudrate = 9600, address = 134)	# Init the Motor
motor2.open()								# Open then connection
print(f"Connection Status: {motor2.saber.is_open}")			# Let us know if it is open
motor2.info()								# Get the motor info

construction_motors = Sabertooth("/dev/serial0", baudrate = 9600, address = 128)	# Init the Motor
construction_motors.open()								# Open then connection
print(f"Connection Status: {construction_motors.saber.is_open}")			# Let us know if it is open
construction_motors.info()								# Get the motor info

LA = linearactuator.linearactuator()		# Init the linear actuator


def linear_motion(speed:int):
	## Motor 1
	motor1.drive(1,speed)	# Turn on motor 1
	motor1.drive(2,speed)	# Turn on motor 2

	## Motor 2
	motor2.drive(1, -speed)	# Turn on motor 1
	motor2.drive(2, -speed)	# Turn on motor 2



def excavate(initial_time):
    lowering_time = 5  # Duration of lowering trencher in seconds
    excavate_time = 10  # Duration of excavation in seconds
    raising_time = 5  # Duration of raising trencher in seconds
    if time.time() - initial_time < lowering_time:
        print("Lowering trencher")
        LA.move(-1)  # Lower the trencher
        return False  # Excavation is still in progress
    elif time.time() - initial_time < (lowering_time + excavate_time):
         LA.stop()
         construction_motors.drive(1, 100)
         construction_motors.drive(2, 10)
         linear_motion(10)  # Move forward while excavating
         print("Excavating")
         return False
    elif time.time() - initial_time < (lowering_time + excavate_time + raising_time):
         print("Raising trencher")
         LA.move(1)  # Raise the trencher
         construction_motors.drive(1, 50)  # continue the excavation motor to deposit remaining regolith
    else: 
        print("Excavation complete")
        LA.stop()
        construction_motors.drive(1, 0)  # Stop the excavation motor
        construction_motors.drive(2, 0)  # Stop the deposition motor
        return True 
    
while True:
    initial_time = time.time()  # Record the start time of the excavation process
    if excavate(initial_time):
        break  # Exit the loop when excavation is complete