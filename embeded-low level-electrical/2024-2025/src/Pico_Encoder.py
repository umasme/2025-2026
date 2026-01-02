'''
Project:    CAN Message Display
Autor:      NotAWildernessExplorer
Date:       03/21/2025

MATERIALS:
 - MCP2515 CAN Bus Module with TJA1050 receiver 
 - Incrimental Encoder
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
|    Encoder  |   Pico  |   
-------------------------
|     3V      |   3.3V  | 
|     5V      |   5V    |
|     Pin3    |   N.C.  |
|     Pin4    |   N.C.  | 
|     B       |   GP15  |  
|     Pin6    |   N.C.  |  
|     A       |   GP14  |   
|     Pin8    |   N.C.  | 
|     Pin9    |   N.C.  | 
|     GND     |   GND   | 
-------------------------


Dependencies:
 - Adafruit_CircuitPython_BusDevice:    https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 - Adafruit_CircuitPython_MCP2515:      https://github.com/adafruit/Adafruit_CircuitPython_MCP2515


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

## System Variables
loop_hold_time = 200_000_000    # [ns] time delay to hold the loop for


## Setup the encoder
enc0 = rotaryio.IncrementalEncoder(board.GP14, board.GP15)
enc1 = rotaryio.IncrementalEncoder(board.GP12, board.GP13)
last_position0 = 0
last_position1 = 0

## Init the SPI Bus
spi = busio.SPI(board.GP18, board.GP19, board.GP16)

## Setup the MCP2515 
cs = digitalio.DigitalInOut(board.GP17)                                                         # Set the CS pin
cs.switch_to_output()                                                                           # Ensure the CS pin is an output
CAN_bus  = CAN(spi, cs, baudrate = 500000, crystal_freq = 8000000,loopback=True, silent=True)   # use loopback to test without another device


## Put all message readout in this function
def read_CAN(count:int)->None:
    '''
    Function will read `count` number of messages in the CAN lisitner Queue
    '''

    for _n in range(count):
        msg = listener.receive()                                             # Read out data
        unpacked_int = struct.unpack('<q',msg.data)                          # Unpack data
        print(f"Recieved packet ({unpacked_int}) from ID {hex(msg.id)}")     # print output over USB

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

    ## STAY. READ ENCODERS. Good boi
    while time.monotonic_ns() - last_send < loop_hold_time: 
        last_position0 = enc0.position
        last_position1 = enc1.position

    ## Send the encoder counts
    with CAN_bus.listen(timeout=1.0) as listener:
        '''!!Code in this block will only run when CAN is active!!'''
        
        ## Pack our int and send
        msg_500 = CAN_bus.send(Message(id=0x500, data=struct.pack('<q',last_position0), extended=False))
        msg_501 = CAN_bus.send(Message(id=0x501, data=struct.pack('<q',last_position1), extended=False))
        
        ## Read CAN messages in queue
        # if (N_msg:=listener.in_waiting()) is not None: read_CAN(N_msg)


    last_send = time.monotonic_ns()
    ## Timing Control on the loop; swap commented lines to clock the max loop timing
    print(f"Loop completed in {(time.monotonic_ns()-lTime)*10**-6:.03f}ms")
