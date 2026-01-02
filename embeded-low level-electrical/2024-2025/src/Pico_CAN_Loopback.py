'''
Project:    MCP2515 + PICO CAN Testing
Autor:      NotAWildernessExplorer
Date:       01/07/2023
Edit:       03/09/2025

MATERIALS:
 - MCP2515 CAN Bus Module with TJA1050 receiver 
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

Dependencies:
 - Adafruit_CircuitPython_BusDevice:    https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 - Adafruit_CircuitPython_MCP2515:      https://github.com/adafruit/Adafruit_CircuitPython_MCP2515

This code proforms a simple loopback test on a Raspberry Pi Pico under loop-back. i.e. only one CAN module is required

Timing test on 30/09/2025 showed an AVG loop time of 17.8ms. Most of that was probably the prints lol.

'''
## Record the boot time
import time
bTime = time.monotonic_ns()    


## Import Libraries
import board
import busio
import struct
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Message
from adafruit_mcp2515 import MCP2515 as CAN


## System Variables
loop_hold_time = 500_000_000    # [ns] time delay to hold the loop for
## Init the SPI Bus
spi = busio.SPI(board.GP18, board.GP19, board.GP16)

## Setup the MCP2515 
cs = DigitalInOut(board.GP17)                                                                   # Set the CS pin
cs.switch_to_output()                                                                           # Ensure the CS pin is an output
CAN_bus  = CAN(spi, cs, baudrate = 500000, crystal_freq = 8000000,loopback=True, silent=True)   # use loopback to test without another device


## Put all message readout in this function
def read_CAN(count:int)->None:
    '''
    Function will read `count` number of messages in the CAN lisitner Queue
    '''

    for _n in range(count):
        msg = listener.receive()
       
        
        if msg.id == 0x501:
            recieve_float, recieve_int = struct.unpack('<fi',msg.data)

            print(f"Recieved packet ({recieve_float},{recieve_int}) from ID {hex(msg.id)}")
    
    return None


## Start the control loop
print(f"RP2040 Setup Completed in {(time.monotonic_ns()-bTime)*10**-6:.01f}ms\nContinuting to loop code")
sTime = time.time() # store the control loop start time
while True:

    '''
    Any Code here will Run, Regardless of CAN Status:
        - ALL SAFETY FUNCTIONS NEED TO BE HERE
        - Any compuations from sensors should be here too
    '''
    lTime = time.monotonic_ns()                     # start of Loop time 

    ## Here I have made two arb numbers to send
    send_float:float    = 3.14159                   # a few digits of pi
    sent_int:int        = int(time.time() - sTime)  # Current uptime of the funcional code to nearest second


    with CAN_bus.listen(timeout=1.0) as listener:
        '''!!Code in this block will only run when CAN is active!!'''
        
        ## Pack our int and float into binary 
        msg_501 = Message(id=0x501, data=struct.pack('<fi',send_float,sent_int), extended=False)

        
        # Send message with verbose
        if CAN_bus.send(msg_501) == True:
            last_send = time.monotonic_ns()
            print(f"Send success from ID {hex(msg_501.id)}")
        

        ## Read CAN messages in queue
        if (N_msg:=listener.in_waiting()) is not None: read_CAN(N_msg)

    
    ## Timing Control on the loop; swap commented lines to clock the max loop timing
    while time.monotonic_ns() - last_send < loop_hold_time: pass
    # print(f"Loop completed in {(time.monotonic_ns()-lTime)*10**-6:.03f}ms")
   