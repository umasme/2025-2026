'''
-----------------------------------------------------
-----------------------------------------------------
Lunabotics Motor Testing
Author: NotAWildernessExplorer
Date:04/16/2025 


-----------------------------------------------------
-----------------------------------------------------
Use MOSFET LVL shifter for UART_TX so the Sabertooth 
can read us

Connect to UART TX
-----------------------------------------------------
-----------------------------------------------------
If access is denied to the serial port, run command
$sudo chmod 666 /dev/serial0

'''

## import library 
import time				# What time is it? Well, this library will tell you!!! 
from pysabertooth import Sabertooth


class DriveTrain():

	def __init__(self):				
		## Init up the sabertooth 1, and open the seral connection 
		self.motor1 = Sabertooth("/dev/serial0", baudrate = 9600, address = 129)	# Init the Motor
		self.motor1.open()								# Open then connection

		## Init up the sabertooth 2, and open the seral connection 
		self.motor2 = Sabertooth("/dev/serial0", baudrate = 9600, address = 134)	# Init the Motor
		self.motor2.open()								# Open then connection
	
	def status(self):
		print(f"Connection Status: {self.motor1.saber.is_open}")			# Let us know if it is open
		self.motor1.info()								# Get the motor info

		print(f"Connection Status: {self.motor2.saber.is_open}")			# Let us know if it is open
		self.motor2.info()								# Get the motor info

	def stop(self):
		self.motor1.stop()
		self.motor2.stop()

	def close(self):
		self.stop()
		del self.motor1
		del self.motor2

	def linear_motion(self,speed:int):

		## Motor 1
		self.motor1.drive(1,speed)	# Turn on motor 1
		self.motor1.drive(2,speed)	# Turn on motor 2
		time.sleep(0.001)
		## Motor 2
		self.motor2.drive(1, -speed)	# Turn on motor 1
		self.motor2.drive(2, -speed)	# Turn on motor 2

	def turn_motion(self,speed:int):
		
		## Motor 1
		self.motor1.drive(1,-speed)	# Turn on motor 1
		self.motor1.drive(2,speed)	# Turn on motor 2

		time.sleep(0.01)

		## Motor 2
		self.motor2.drive(1,-speed)	# Turn on motor 1
		self.motor2.drive(2,speed)	# Turn on motor 2



class CheeseGrater():
	def __init__(self):
		## Init up the sabertooth 2, and open the seral connection 
		self.motor3 = Sabertooth("/dev/serial0", baudrate = 9600, address = 128)	# Init the Motor
		self.motor3.open()								# Open then connection

	def status(self):
		print(f"Connection Status: {self.motor3.saber.is_open}")			# Let us know if it is open
		self.motor3.info()								# Get the motor info

	def stop(self):
		self.motor3.stop()

	def close(self):
		self.stop()
		del self.motor3

	def dig(self,speed:int):
		self.motor3.drive(1, speed)

	def deposition(self):
		self.motor3.drive(2, -100)



