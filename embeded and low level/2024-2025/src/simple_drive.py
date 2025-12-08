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
from sshkeyboard import listen_keyboard
import time
from pysabertooth import Sabertooth


# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
# -----------------------Set up sabers-----------------------------------
# -----------------------------------------------------------------------
# -----------------------------------------------------------------------


## Init up the sabertooth 1, and open the seral connection 
motor1 = Sabertooth("/dev/serial0", baudrate = 9600, address = 129)	# Init the Motor
motor1.open()								# Open then connection
print(f"Connection Status: {motor1.saber.is_open}")			# Let us know if it is open
motor1.info()								# Get the motor info


## Init up the sabertooth 2, and open the seral connection 
motor2 = Sabertooth("/dev/serial0", baudrate = 9600, address = 134)	# Init the Motor
motor2.open()								# Open then connection
print(f"Connection Status: {motor2.saber.is_open}")			# Let us know if it is open
motor2.info()								# Get the motor info

## Init up the sabertooth 2, and open the seral connection 
motor3 = Sabertooth("/dev/serial0", baudrate = 9600, address = 128)	# Init the Motor
motor3.open()								# Open then connection
print(f"Connection Status: {motor3.saber.is_open}")			# Let us know if it is open
motor3.info()								# Get the motor info


# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
# -----------------------Helper Functions--------------------------------
# -----------------------------------------------------------------------
# -----------------------------------------------------------------------


# Key release function
def release(key):
     print("Motors Stopped")
     stop_all()

# Key press function
def press(key):
    lin_speed:int  = 70
    turn_speed:int = 30
    print(f"{key} pressed")
    print(type(key))
    key = key.lower()
    
    if  key == 'w':
    	linear_motion(lin_speed)
    elif key == 's':
    	linear_motion(-lin_speed)
    elif key == 'a':
    	turn_motion(-turn_speed)
    elif key == 'd':
    	turn_motion(turn_speed)
    elif key == "q":
    	deposit()
    else: 
        print("NOT A DIRRECTINNO, N3RD!") 
    

# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
# ----------------------Motor Controls-----------------------------------
# -----------------------------------------------------------------------
# -----------------------------------------------------------------------

def stop_all():
	motor1.stop()			# Turn off both motors
	motor2.stop()			# Turn off both motors
	motor3.stop()

def linear_motion(speed:int):
	print("Move")
	## Motor 1
	motor1.drive(1,speed)	# Turn on motor 1
	motor1.drive(2,speed)	# Turn on motor 2

	time.sleep(0.01)

	## Motor 2
	motor2.drive(1, -speed)	# Turn on motor 1
	motor2.drive(2, -speed)	# Turn on motor 2



def turn_motion(speed:int):
	
	## Motor 1
	motor1.drive(1,-speed)	# Turn on motor 1
	motor1.drive(2,speed)	# Turn on motor 2

	time.sleep(0.01)

	## Motor 2
	motor2.drive(1,-speed)	# Turn on motor 1
	motor2.drive(2,speed)	# Turn on motor 2

def deposit():
	motor3.drive(2, 100)






if __name__ == '__main__':

        while True:
               listen_keyboard(on_press=press, on_release=release, sequential=True)
               time.sleep(0.01)
