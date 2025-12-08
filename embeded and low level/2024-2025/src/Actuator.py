'''
-----------------------------------------------------
-----------------------------------------------------
Lunabotics Linear Actuator Testing
Author: NotAWildernessExplorer
Date:04/11/2025 


-----------------------------------------------------
-----------------------------------------------------
Conenctions for BST7960 ot Pi4 b

Pin 1:	GP12 pin 32
Pin 2:	GP13 pin 33
Pin 3:	3V
Pin 4:	3V
Pin 5:	NC
Pin 6: 	NC
Pin 7:	3V
Pin 8:	GND

-----------------------------------------------------
-----------------------------------------------------


'''

## import library 
import time				# What time is it? Well, this library will tell you!!! 
import RPi.GPIO as GPIO

## Set pi4b to use Board pin numbers
GPIO.setmode(GPIO.BOARD)

## Luna linear actuators start here!
class linearactuator():
    def __init__(self):

        GPIO.setup(32,GPIO.OUT)                 # set up pin GPIO 12 pin 32
        GPIO.setup(33,GPIO.OUT)                 # set up pin GPIO 13 pin 33
        self.R_PWM = GPIO.PWM(33,125_000_000)   # Init pin 32 as pwm at 125 MHz
        self.L_PWM = GPIO.PWM(32,125_000_000)   # init pin 33 as pwm at 125 MHz
        self.R_PWM.start(0)                     # start R pwm channel
        self.L_PWM.start(0)                     # start L pwm channel
        
    def move(self,qty):
        '''
        Changes  motor controller duty cycle\n
        qty > 0: extend \n
        qty < 0: retract \n
        qty = 0: stop

        '''
        if qty > 0:
            self.stop()                         # Stop motors
            time.sleep(0.001)                   # wait
            self.R_PWM.ChangeDutyCycle(100)     # change duty cycle
        elif qty < 0:
            self.stop()                         # Stop motors
            time.sleep(0.001)                   # wait
            self.L_PWM.ChangeDutyCycle(100)     # change duty cycle
        else:
            self.stop()                         # Stop motors
            time.sleep(0.001)                   # wait
        

    def stop(self):
        '''stops the motors'''
        self.R_PWM.ChangeDutyCycle(0)           # Set forward pwm to zero
        self.L_PWM.ChangeDutyCycle(0)           # set reverse pwm to zero



