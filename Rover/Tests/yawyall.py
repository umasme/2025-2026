#!/usr/bin/env python3

'''
Project:    Rover Yaw Control
Autor:      NotAWildernessExplorer
Date:       05/15/2025



Install Depth-Ai https://docs.luxonis.com/software/depthai/manual-install/

The following is baised in part on the luxonis gyro/accelerometer example
https://docs.luxonis.com/software/depthai/examples/imu_accelerometer_gyroscope/

'''
import cv2
import depthai as dai
import time
import math
import numpy as np



'''
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-----------------------Fancy Functions to Copy----------------------------------
--------------------------------------------------------------------------------
'''

def integrate_angles(current_angle:np.ndarray[float],packets,history_vars:list):
    '''
    current_angle:np.ndarrray[float] The angle you want to intergrate forward in time. must be of len 3
    \nacceleroValues: the imu accelerometer packet
    \ngyroValues: the imu gyroscope packet
    \nhistory_vars: list of [base timestep,previous gyro timestep, previous gyro reading after rotation]
    \n
    \nInitalization of history_vars should be: [None,None,np.array([0.0,0.0,0.0])]
    '''
    imuPackets = packets                # Collect the packets from the queue
    for imuPacket in imuPackets:        # Loop over all packets

        ## Grab the accelerometer and gyro info from the packet      
        acceleroValues = imuPacket.acceleroMeter
        gyroValues = imuPacket.gyroscope
        
        # Unpack the history information
        baseTs, prev_gyroTs,prev_gyro_rotated = history_vars
        angles = current_angle
        
        acceleroTs = acceleroValues.getTimestampDevice()
        gyroTs = gyroValues.getTimestampDevice()



        ## Musss code for timing interval dt[1]
        acceleroTs = acceleroValues.getTimestampDevice()
        gyroTs = gyroValues.getTimestampDevice()

        if baseTs is None:
            baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
            prev_gyroTs = gyroTs
            print(baseTs)
            
            history_vars = [baseTs,prev_gyroTs,prev_gyro_rotated]
            return angles,history_vars

        acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
        gyroTs = timeDeltaToMilliS(gyroTs - baseTs)


        ## Calculate the time difference between the current and previous gyroscope readings
        dt_gyro = (gyroTs - timeDeltaToMilliS(prev_gyroTs - baseTs)) / 1000.0   # Convert milliseconds to seconds
        prev_gyroTs = gyroValues.getTimestampDevice()                           # Save for next time
            
        ## end of muss code
        

        ## Do not disturb the sleeping dragon!
        g_accel = np.array([-acceleroValues.x,acceleroValues.y,acceleroValues.z])
        g_mag = np.sqrt(np.dot(g_accel,g_accel))
        g_norm = g_accel/g_mag

        ##Rotate about X
        theta_x = -np.arctan2((g_norm[2]),(g_norm[1]))
        rot_matrix_x = np.array([[1,0,0],[0,np.cos(theta_x),-np.sin(theta_x)],[0,np.sin(theta_x),np.cos(theta_x)]])  
        g_norm = rot_matrix_x.dot(g_norm)

        ## Rotate about Z
        theta_z = np.arctan2((g_norm[0]),(g_norm[1]))
        rot_matrix_z = np.array([[np.cos(theta_z),-np.sin(theta_z),0],[np.sin(theta_z),np.cos(theta_z),0],[0,0,1]])
        g_norm = rot_matrix_z.dot(g_norm)

        ## Roate Gyro
        gyro = np.array([gyroValues.x,gyroValues.y,gyroValues.z])
        gyro_rotated = rot_matrix_z.dot(rot_matrix_x.dot(gyro))



        ## Integrate Angle with trapizodial rule for variable step size
        angles += ((gyro_rotated + prev_gyro_rotated)/2 * dt_gyro)*180/np.pi        #
        prev_gyro_rotated = gyro_rotated                                #
        

        ## Pack up the history
        history_vars = [baseTs,prev_gyroTs,prev_gyro_rotated]


        ## Print Time
        print(f"{gyroTs:.0f},",end="")

        ## DBG: To check if gravity is +1 in y dirrection only
        printvec(g_norm,ending=',')

        ## DBG: To check if angles are integrating correctly
        printvec(angles)


    ## Return the integrated angles and the history
    return angles,history_vars


def printvec(vec,ending = '\n'):
    '''Because Russell Likes the way it looks in terminal'''
    ps1 = ("+" if vec[0] > 0 else"-") +f"{abs(vec[0]):.2f}," 
    ps2 = ("+" if vec[1] > 0 else"-") +f"{abs(vec[1]):.2f}," 
    ps3 = ("+" if vec[2] > 0 else"-") +f"{abs(vec[2]):.2f}"
    print(ps1+ps2+ps3,end=ending)

def timeDeltaToMilliS(delta) -> float:
    return delta.total_seconds()*1000

if __name__ == "__main__":
    ## The following globals must be defined
    angle_history = [None,None,np.array([0.0,0.0,0.0])]
    angles = np.array([0.0,0.0,0.0])

    '''
    --------------------------------------------------------------------------------
    --------------------------------------------------------------------------------
    ---------------------------Set Up Cameras---------------------------------------
    --------------------------------------------------------------------------------
    This was straight up coppied from the source
    '''

    device = dai.Device()

    imuType = device.getConnectedIMU()
    imuFirmwareVersion = device.getIMUFirmwareVersion()
    print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")



    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    xlinkOut.setStreamName("imu")

    # enable ROTATION_VECTOR at 400 hz rate
    #imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)

    # Enable ACCELEROMETER_RAW at 500 Hz rate
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
    # Enable GYROSCOPE_RAW at 400 Hz rate
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # if lower or equal to batchReportThreshold then the sending is always blocking on device
    # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(10)

    # Link plugins IMU -> XLINK
    imu.out.link(xlinkOut.input)


    '''
    --------------------------------------------------------------------------------
    --------------------------------------------------------------------------------
    ---------------------------Readout Loop-----------------------------------------
    --------------------------------------------------------------------------------
    '''

    print("Starting\n...")
    # Pipeline is defined, now we can connect to the device
    with device:
        device.startPipeline(pipeline)

        # Output queue for imu bulk packets
        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

        while True:
            imuData = imuQueue.get()            # blocking call, will wait until a new data has arrived

            imuPackets = imuData.packets        # Collect the packets from the queue
            
            ## This is the line that updates the angles Alex
            angles,angle_history = integrate_angles(angles,imuPackets,angle_history)
            


    # References
    # [1] Chat GPT