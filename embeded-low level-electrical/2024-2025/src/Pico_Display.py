'''
Project:    CAN Message Display
Autor:      NotAWildernessExplorer
Date:       03/20/2025

MATERIALS:
 - MCP2515 CAN Bus Module with TJA1050 receiver 
 - 2x16 MonoChromatic Character Display
 - Pot (For Backlight)
 - Raspberry Pi Pico
 - Jumper wires for connections

Wiring:
-------------------------
|    MCP2515  |   Pico  |   
-------------------------
|     MOSI    |   GP19  | 
|     SCK     |   GP18  |
|     MISO    |   GP16  |
|     CS      |   GP17  | 
|     VCC     |   3V3   | 
|     GND     |   GND   | 
-------------------------


-------------------------
|    Char Dsp |   Pico  |   
-------------------------
|     Pin 01  |   GND   | 
|     Pin 02  |   +5V   | 
|     Pin 03  |   pot   | 
|     Pin 04  |   GP20  | 
|     Pin 05  |   GND   | 
|     Pin 06  |   GP21  | 
|     Pin 07  |   N.C.  | 
|     Pin 08  |   N.C.  | 
|     Pin 09  |   N.C.  | 
|     Pin 10  |   GP22  | 
|     Pin 11  |   GP26  | 
|     Pin 12  |   GP27  | 
|     Pin 13  |   GP28  | 
|     Pin 14  |   +5V   | 
|     Pin 15  |   GND   | 
-------------------------

Dependencies:
 - Adafruit_CircuitPython_BusDevice:    https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 - Adafruit_CircuitPython_MCP2515:      https://github.com/adafruit/Adafruit_CircuitPython_MCP2515
 - Adafruit_CircuitPython_Character_LCD:
 - Adafruit_CircuitPython_MCP230xx:
 - Adafruit_CircuitPython_74hc595:

This code reads the can Line and outputs the message of know id's (i.e. the ones in read_can() ) to the display


'''
## Record the boot time
import time
bTime = time.monotonic_ns()    


## Import Libraries
import board
import busio
import struct
import digitalio
import rotaryio
from adafruit_mcp2515 import MCP2515 as CAN
from adafruit_mcp2515.canio import Message
import adafruit_character_lcd.character_lcd as characterlcd

## System Variables
loop_hold_time = 500_000_000    # [ns] time delay to hold the loop for


## Setup the encoder
enc = rotaryio.IncrementalEncoder(board.GP14, board.GP15)
last_position = None

## Init the SPI Bus
spi = busio.SPI(board.GP18, board.GP19, board.GP16)

## Setup the MCP2515 
cs = digitalio.DigitalInOut(board.GP17)                                                         # Set the CS pin
cs.switch_to_output()                                                                           # Ensure the CS pin is an output
CAN_bus  = CAN(spi, cs, baudrate = 500000, crystal_freq = 8000000,loopback=True, silent=True)   # use loopback to test without another device

## Setup the Display pins
lcd_rs = digitalio.DigitalInOut(board.GP20)
lcd_en = digitalio.DigitalInOut(board.GP21)
lcd_d7 = digitalio.DigitalInOut(board.GP28)
lcd_d6 = digitalio.DigitalInOut(board.GP27)
lcd_d5 = digitalio.DigitalInOut(board.GP26)
lcd_d4 = digitalio.DigitalInOut(board.GP22)

## Set number of characters
lcd_columns = 16
lcd_rows    = 2

## Create LCD object
lcd = characterlcd.Character_LCD_Mono(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows)
lcd.clear()

## Play a little welcome message
lcd.blink = True
lcd.message = "Hello\nLunaBotics!"
time.sleep(2.0)
lcd.blink = False
for i in range(16):
    lcd.move_left()
    time.sleep(0.2)
lcd.clear()


## Put all message readout in this function
def read_CAN(count:int)->None:
    '''
    Function will read `count` number of messages in the CAN lisitner Queue
    '''

    for _n in range(count):
        msg = listener.receive()
       
        
        if msg.id == 0x501:
            recieve_float, recieve_int = struct.unpack('<fi',msg.data)                          # Unpack data
            print(f"Recieved packet ({recieve_float},{recieve_int}) from ID {hex(msg.id)}")     # print output over USB
            lcd.message = f"From ID {hex(msg.id)}:\n({recieve_float:.2f},{recieve_int})"        # send message to dispaly

    return None






## Start the control loop
print(f"RP2040 Setup Completed in {(time.monotonic_ns()-bTime)*10**-6:.01f}ms\nContinuting to loop code")
sTime = time.time() # store the control loop start time
lcd.message = "Starting CAN"
time.sleep(1.5)
lcd.clear()





# while True:
#     position = enc.position
#     if last_position == None or position != last_position:
#         print(position*360/256)
#     last_position = position
## This loop preforms loopback testing on singular pico+can for address x501
while True:
    '''
    Any Code here will Run, Regardless of CAN Status:
        - ALL SAFETY FUNCTIONS NEED TO BE HERE
        - Any compuations from sensors should be here too
    '''
    lTime = time.monotonic_ns()                     # start of Loop time 


    with CAN_bus.listen(timeout=1.0) as listener:
        '''!!Code in this block will only run when CAN is active!!'''
        
        ## Pack our int and float into binary 
        msg_501 = Message(id=0x501, data=struct.pack('<fi',enc.position*360/256,enc.position), extended=False)
        
        # Send message with verbose
        if CAN_bus.send(msg_501) == True:
            last_send = time.monotonic_ns()
            #print(f"Send success from ID {hex(msg_501.id)}")
        

        ## Read CAN messages in queue
        #if (N_msg:=listener.in_waiting()) is not None: read_CAN(N_msg)

    
    ## Timing Control on the loop; swap commented lines to clock the max loop timing
    #while time.monotonic_ns() - last_send < loop_hold_time: pass
    print(f"Loop completed in {(time.monotonic_ns()-lTime)*10**-6:.03f}ms")
