## NEEDS COMMENTS

import os
from ControlTable import *
from dynamixel_sdk import *
import numpy as np
import copy as cp
import time
from curtsies import Input
from threading import Thread
import RPi.GPIO as GPIO

def InitialSetup():
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    return portHandler, packetHandler

def gcd(a, b): 
    if b == 0: 
        return a 
    else: 
        return gcd(b, a%b) 
   
def RotatePositionArray(inArray,shiftNum,arrayLength):
    for i in range(gcd(shiftNum,arrayLength)):  
        temp = inArray[i] 
        j = i 
        while 1: 
            k = j + shiftNum 
            if k >= arrayLength: 
                k = k - arrayLength 
            if k == i: 
                break
            inArray[j] = inArray[k] 
            j = k 
        inArray[j] = temp
    return inArray

def DataAddrConversion(DesiredData):
    DataAddr = -1
    # Get the address of the desired data trait, used in conjuction with ReturnRelevantData and ChangeSpecificTrait
    if (DesiredData >= 1 and DesiredData <= 48):
        DataAddr = AddrDict[DesiredData]
    else:
        print("That's not a recognized trait selection, please try again!\n")
    return DataAddr if DataAddr is not -1 else -1

def FormatSendData(rawData):
    return [DXL_LOBYTE(DXL_LOWORD(rawData)), DXL_HIBYTE(DXL_LOWORD(rawData)), DXL_LOBYTE(DXL_HIWORD(rawData)), DXL_HIBYTE(DXL_HIWORD(rawData))]

def DetectStopInput():
    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            if (str(e) == '\x1b'):
                print("Stopping Servo Movement\n")
                global stopVal
                stopVal = 1
                break

def DetermineSpeeds(tspan,positionsFile):
    import numpy as np
    import pandas as pd
    import math
    import time
    import copy as cp
    import sys
    #Import position points as a data frame (df)
    old_df = pd.read_csv(positionsFile)
    old_df = old_df.values[:][:]
    #Make a copy of the dataframe with the same dimensions for the speeds
    speeds = cp.copy(old_df)
    h_stance = 4.892994
    h_swing = 3.347006
    f_stance = 5.211718
    f_swing = 3.028282
    h_st_per = h_stance / 8.24
    h_sw_per = h_swing / 8.24
    f_st_per = f_stance / 8.24
    f_sw_per = f_swing / 8.24
    b = speeds.shape
    cLen = b[0]
    cLength = cLen/2
    cWidth = b[1]
    percents = np.linspace(1,cLen-1,cLen-1)
    joints = list(range(1,cWidth+1))
    percents = percents.astype(int).tolist()
    for i in percents:
        for j in joints:
            if (j == 1 or j == 2 or j == 3 or j == 4 ):
                rotations = abs(speeds[i][j-1]-speeds[i-1][j-1])/4096
                movementTime = (tspan*f_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                speeds[i-1][j-1] = round(movementSpeed,5)
            elif (j == 5 or j == 6 or j == 7 or j == 8): 
                rotations = abs(speeds[i][j-1]-speeds[i-1][j-1])/4096
                movementTime = (tspan*f_sw_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                speeds[i-1][j-1] = round(movementSpeed,5)
            elif (j == 9 or j == 10 or j == 11 or j == 12 ):
                rotations = abs(speeds[i][j-1]-speeds[i-1][j-1])/4096
                movementTime = (tspan*h_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                speeds[i-1][j-1] = round(movementSpeed,5)
            elif (j == 13 or j == 14 or j == 15 or j == 16 ):
                rotations = abs(speeds[i][j-1]-speeds[i-1][j-1])/4096
                movementTime = (tspan*h_sw_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                speeds[i-1][j-1] = round(movementSpeed,5)
    speeds = np.round(speeds)
    joints = list(range(0,cWidth))
    for j in joints:
        if (j == 0 or j == 1 or j == 2 or j == 3):
            rotations = abs(old_df[-1][j] - old_df[0][j+4])/4096
            movementTime = (tspan * f_st_per / cLength) / 60
            movementSpeed = (rotations / movementTime) / 0.114
            speeds[-1][j] = round(movementSpeed)
        elif (j == 4 or j == 5 or j == 6 or j == 7):
            rotations = abs(old_df[-1][j] - old_df[0][j-4])/4096
            movementTime = (tspan * f_sw_per / cLength) / 60
            movementSpeed = (rotations / movementTime) / 0.114
            speeds[-1][j] = round(movementSpeed)
        elif (j == 8 or j == 9 or j == 10 or j == 11):
            rotations = abs(old_df[-1][j] - old_df[0][j+4])/4096
            movementTime = (tspan * h_st_per / cLength) / 60
            movementSpeed = (rotations / movementTime) / 0.114
            speeds[-1][j] = round(movementSpeed)
        elif (j == 12 or j == 13 or j == 14 or j == 15):
            rotations = abs(old_df[-1][j] - old_df[0][j-4])/4096
            movementTime = (tspan * h_sw_per / cLength) / 60
            movementSpeed = (rotations / movementTime) / 0.114
            speeds[-1][j] = round(movementSpeed)
    speeds[speeds==0]=1
    rows = speeds.shape[0]
    cols = speeds.shape[1]
    for x in range(0, rows):
        for y in range(0, cols):
            if (speeds[x][y] > 1023):
                speeds[x][y] = 1023
    return speeds

def PostProcessSpeeds(speeds):
    FL_1_Stance = speeds[:,0]
    FL_1_Swing = speeds[:,4]
    FL_2_Stance = speeds[:,1]
    FL_2_Swing = speeds[:,5]
    FL_3_Stance = speeds[:,2]
    FL_3_Swing = speeds[:,6]
    FL_4_Stance = speeds[:,3]
    FL_4_Swing = speeds[:,7]
    
    HL_1_Stance = speeds[:,8]
    HL_1_Swing = speeds[:,12]
    HL_2_Stance = speeds[:,9]
    HL_2_Swing = speeds[:,13]
    HL_3_Stance = speeds[:,10]
    HL_3_Swing = speeds[:,14]
    HL_4_Stance = speeds[:,11]
    HL_4_Swing = speeds[:,15]

    # FL_1_Stance = np.array(FL_1_Stance)
    # FL_2_Stance = np.array(FL_2_Stance)
    # FL_3_Stance = np.array(FL_3_Stance)
    # FL_4_Stance = np.array(FL_4_Stance)

    # HL_1_Stance = np.array(HL_1_Stance)
    # HL_2_Stance = np.array(HL_2_Stance)
    # HL_3_Stance = np.array(HL_3_Stance)
    # HL_4_Stance = np.array(HL_4_Stance)

    # FL_1_Swing = np.array(FL_1_Swing)
    # FL_2_Swing = np.array(FL_2_Swing)
    # FL_3_Swing = np.array(FL_3_Swing)
    # FL_4_Swing = np.array(FL_4_Swing)

    # HL_1_Swing = np.array(HL_1_Swing)
    # HL_2_Swing = np.array(HL_2_Swing)
    # HL_3_Swing = np.array(HL_3_Swing)
    # HL_4_Swing = np.array(HL_4_Swing)

    FL_1_Stride = np.concatenate((FL_1_Stance,FL_1_Swing),axis=0)
    FL_2_Stride = np.concatenate((FL_2_Stance,FL_2_Swing),axis=0)
    FL_3_Stride = np.concatenate((FL_3_Stance,FL_3_Swing),axis=0)
    FL_4_Stride = np.concatenate((FL_4_Stance,FL_4_Swing),axis=0)

    HL_1_Stride = np.concatenate((HL_1_Stance,HL_1_Swing),axis=0)
    HL_2_Stride = np.concatenate((HL_2_Stance,HL_2_Swing),axis=0)
    HL_3_Stride = np.concatenate((HL_3_Stance,HL_3_Swing),axis=0)
    HL_4_Stride = np.concatenate((HL_4_Stance,HL_4_Swing),axis=0)


    # Shift Number for limb #1 is shifted up by 1 (not present here, number becomes 2 instead of 1)
    ServoVel1 = RotatePositionArray(FL_1_Stride,1,len(FL_1_Stride)) 
    ServoVel2 = RotatePositionArray(FL_2_Stride,1,len(FL_2_Stride))
    ServoVel3 = RotatePositionArray(FL_3_Stride,1,len(FL_3_Stride))
    ServoVel4 = RotatePositionArray(FL_4_Stride,1,len(FL_4_Stride))

    ServoVel5 = RotatePositionArray(FL_1_Stride,9,len(FL_1_Stride))
    ServoVel6 = RotatePositionArray(FL_2_Stride,9,len(FL_2_Stride))
    ServoVel7 = RotatePositionArray(FL_3_Stride,9,len(FL_3_Stride))
    ServoVel8 = RotatePositionArray(FL_4_Stride,9,len(FL_4_Stride))

    ServoVel9 = RotatePositionArray(HL_1_Stride,8,len(HL_1_Stride)) 
    ServoVel10 = RotatePositionArray(HL_2_Stride,8,len(HL_2_Stride))
    ServoVel11 = RotatePositionArray(HL_3_Stride,8,len(HL_3_Stride))
    ServoVel12 = RotatePositionArray(HL_4_Stride,8,len(HL_4_Stride))

    # Does Not Need to be Rotated, starts exactly at zero (0)
    ServoVel13 = HL_1_Stride
    ServoVel14 = HL_2_Stride
    ServoVel15 = HL_3_Stride
    ServoVel16 = HL_4_Stride

    ServoVel1 = ServoVel1.reshape(22,1)
    ServoVel2 = ServoVel2.reshape(22,1)
    ServoVel3 = ServoVel3.reshape(22,1)
    ServoVel4 = ServoVel4.reshape(22,1)

    ServoVel5 = ServoVel5.reshape(22,1)
    ServoVel6 = ServoVel6.reshape(22,1)
    ServoVel7 = ServoVel7.reshape(22,1)
    ServoVel8 = ServoVel8.reshape(22,1)

    ServoVel9 = ServoVel9.reshape(22,1)
    ServoVel10 = ServoVel10.reshape(22,1)
    ServoVel11 = ServoVel11.reshape(22,1)
    ServoVel12 = ServoVel12.reshape(22,1)

    ServoVel13 = ServoVel13.reshape(22,1)
    ServoVel14 = ServoVel14.reshape(22,1)
    ServoVel15 = ServoVel15.reshape(22,1)
    ServoVel16 = ServoVel16.reshape(22,1)

    FL_VEL = np.concatenate((ServoVel1, ServoVel2, ServoVel3, ServoVel4, ServoVel5, ServoVel6, ServoVel7, ServoVel8),axis=1)
    HL_VEL = np.concatenate((ServoVel9, ServoVel10, ServoVel11, ServoVel12, ServoVel13, ServoVel14, ServoVel15, ServoVel16),axis=1)
    TOT_VEL = np.concatenate(FL_VEL,HL_VEL,axis=1)
    
    return TOT_VEL

def PostProcessPositions():
    ServoPos1 = RotatePositionArray(FL_TOT_R_1,1,len(FL_TOT_R_1))
    ServoPos2 = RotatePositionArray(FL_TOT_R_2,1,len(FL_TOT_R_2))
    ServoPos3 = RotatePositionArray(FL_TOT_R_3,1,len(FL_TOT_R_3))
    ServoPos4 = RotatePositionArray(FL_TOT_R_4,1,len(FL_TOT_R_4))

    ServoPos5 = RotatePositionArray(FL_TOT_L_1,9,len(FL_TOT_L_1))
    ServoPos6 = RotatePositionArray(FL_TOT_L_2,9,len(FL_TOT_L_2))
    ServoPos7 = RotatePositionArray(FL_TOT_L_3,9,len(FL_TOT_L_3))
    ServoPos8 = RotatePositionArray(FL_TOT_L_4,9,len(FL_TOT_L_4))

    ServoPos9 = RotatePositionArray(HL_TOT_R_1,8,len(HL_TOT_R_1))
    ServoPos10 = RotatePositionArray(HL_TOT_R_2,8,len(HL_TOT_R_2))
    ServoPos11 = RotatePositionArray(HL_TOT_R_3,8,len(HL_TOT_R_3))
    ServoPos12 = RotatePositionArray(HL_TOT_R_4,8,len(HL_TOT_R_4))

    # Does Not Need to be Rotated, starts exactly at zero (0)
    ServoPos13 = HL_TOT_L_1
    ServoPos14 = HL_TOT_L_2
    ServoPos15 = HL_TOT_L_3
    ServoPos16 = HL_TOT_L_4

    ServoPos1 = np.array(ServoPos1)
    ServoPos2 = np.array(ServoPos2)
    ServoPos3 = np.array(ServoPos3)
    ServoPos4 = np.array(ServoPos4)

    ServoPos5 = np.array(ServoPos5)
    ServoPos6 = np.array(ServoPos6)
    ServoPos7 = np.array(ServoPos7)
    ServoPos8 = np.array(ServoPos8)

    ServoPos9 = np.array(ServoPos9)
    ServoPos10 = np.array(ServoPos10)
    ServoPos11 = np.array(ServoPos11)
    ServoPos12 = np.array(ServoPos12)

    ServoPos13 = np.array(ServoPos13)
    ServoPos14 = np.array(ServoPos14)
    ServoPos15 = np.array(ServoPos15)
    ServoPos16 = np.array(ServoPos16)

    ServoPos1 = ServoPos1.reshape(22,1)
    ServoPos2 = ServoPos2.reshape(22,1)
    ServoPos3 = ServoPos3.reshape(22,1)
    ServoPos4 = ServoPos4.reshape(22,1)

    ServoPos5 = ServoPos5.reshape(22,1)
    ServoPos6 = ServoPos6.reshape(22,1)
    ServoPos7 = ServoPos7.reshape(22,1)
    ServoPos8 = ServoPos8.reshape(22,1)

    ServoPos9 = ServoPos9.reshape(22,1)
    ServoPos10 = ServoPos10.reshape(22,1)
    ServoPos11 = ServoPos11.reshape(22,1)
    ServoPos12 = ServoPos12.reshape(22,1)

    ServoPos13 = ServoPos13.reshape(22,1)
    ServoPos14 = ServoPos14.reshape(22,1)
    ServoPos15 = ServoPos15.reshape(22,1)
    ServoPos16 = ServoPos16.reshape(22,1)

    FL_TOT = np.concatenate((ServoPos1, ServoPos2, ServoPos3, ServoPos4, ServoPos5, ServoPos6, ServoPos7, ServoPos8),axis=1)
    HL_TOT = np.concatenate((ServoPos9, ServoPos10, ServoPos11, ServoPos12, ServoPos13, ServoPos14, ServoPos15, ServoPos16),axis=1)

    return FL_TOT, HL_TOT

def PrintUserMenu():
    print("---------MENU---------")
    print("1: Move a single servo")
    print("2: Move a specific limb")
    print("3: Move the entire robot")
    print("4: Other")
    print("5: Exit\n")

def PrintOptionsSubmenu():
    print("\n---------SUBMENU---------")
    print("1: Turn Torque setting off for all servos")
    print("2: Turn Torque setting on for all servos")
    print("3: Turn Torque setting off for specific servo")
    print("4: Turn Torque setting on for specific servo")
    print("5: Get servo trait")
    print("6: Send servo trait")
    print("7: Ping all connected servos")
    print("8: Reboot all servos")
    print("9: Reset all servos")
    print("10: Reboot specified servo(s)")
    print("11: Reset specified servo(s)")
    print("12: Test ping outer apparatuses\n")

def DisplayServoTraits():
    print("Servo Traits:\n")
    print("1: Model Number")
    print("2: Model Information")
    print("3: Firmware Version")
    print("4: ID")
    print("5: Baud Rate")
    print("6: Return Delay Time")
    print("7: Drive Mode")
    print("8: Operating Mode")
    print("9: Protocol Type")
    print("10: Homing Offset")
    print("11: Moving Threshold")
    print("12: Temperature Limit")
    print("13: Max Voltage Limit")
    print("14: Min Voltage Limit")
    print("15: PWM Limit")
    print("16: Current Limit")
    print("17: Acceleration Limit")
    print("18: Velocity Limit")
    print("19: Max Position Limit")
    print("20: Min Position Limit")
    print("21: Shutdown")
    print("22: Torque Toggle")
    print("23: LED")
    print("24: Status Return Level")
    print("25: Registered Instruction")
    print("26: Hardware Error Status")
    print("27: Velocity I Gain")
    print("28: Velocity P Gain")
    print("29: Position D Gain")
    print("30: Position I Gain")
    print("31: Position P Gain")
    print("32: Goal PWM")
    print("33: Goal Current")
    print("34: Goal Velocity")
    print("35: Profile Acceleration")
    print("36: Profile Velocity")
    print("37: Goal Position")
    print("38: Realtime Tick")
    print("39: Moving")
    print("40: Moving Status")
    print("41: Present PWM")
    print("42: Present Current")
    print("43: Present Velocity")
    print("44: Present Position")
    print("45: Velocity Trajectory")
    print("46: Position Trajectory")
    print("47: Present Input Voltage")
    print("48: Present Temperature\n")

def PingServos():
    import os

    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Try to broadcast ping the Dynamixel
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    print("Detected Dynamixel :")
    for dxl_id in dxl_data_list:
        print("[ID:%03d] model version : %d | firmware version : %d" % (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))

    # Close port
    portHandler.closePort() 

    return dxl_data_list
    
def CleanUp(BodyObj,LimbObjList,ServoObjList,CurrentDoc):
    for each_servo in ServoObjList:
        each_servo.ToggleTorque(0,each_servo.portHandler,each_servo.packetHandler)
    BodyObj.__del__()
    for each_limb in LimbObjList:
        each_limb.__del__()
    for each_servo in ServoObjList:
        each_servo.__del__()
    if CurrentDoc != None:
        CurrentDoc.CloseDoc()
        CurrentDoc.__del__()

def ShutDown():
    print("Shutting down system.\n")
    # Turn off power to external boards and other systems
    print("Thank you for using Theo!")

def SpeedMerge():
    desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
    speeds = DetermineSpeeds(desired_timespan,PositionsFile)
    TotMatrix_speeds = PostProcessSpeeds(speeds)
    return TotMatrix_speeds

def AssembleRobot(PositionsArray):
    ServoObjList = []
    ServoObjDict = {}
    dxl_data_list = PingServos()
    for dxl_id in dxl_data_list:
        print("[ID:%03d] Detected" % (dxl_id))
        RelativeServo = Servo(dxl_id,PositionsArray[:][dxl_id])
        ServoObjList.append(RelativeServo)
        ServoObjDict[dxl_id] = RelativeServo

    #NumberOfServosConnected = len(dxl_data_list)

    FR_limbCount = 0
    FR_Limb = []
    FL_limbCount = 0
    FL_Limb = []
    BR_limbCount = 0
    BR_Limb = []
    BL_limbCount = 0
    BL_Limb = []
    Neck_limbCount = 0
    Neck_Limb = []
    Spine_limbCount = 0
    Spine_Limb = []
    Tail_limbCount = 0
    Tail_Limb = []
    TheoLimbList = []
    TheoLimbDict = {}
    for ServoID,ServoObj in ServoObjDict:
        if (ServoID == 1 or ServoID == 2 or ServoID == 3 or ServoID == 4):
            FR_limbCount += 1
        elif (ServoID == 5 or ServoID == 6 or ServoID == 7 or ServoID == 8):
            FL_limbCount += 1
        elif (ServoID == 9 or ServoID == 10 or ServoID == 11 or ServoID == 12):
            BR_limbCount += 1
        elif (ServoID == 13 or ServoID == 14 or ServoID == 15 or ServoID == 16):
            BL_limbCount += 1
        elif (ServoID == 17 or ServoID == 18):
            Neck_limbCount += 1
        elif (ServoID == 19 or ServoID == 20 or ServoID == 21 or ServoID == 22):
            Spine_limbCount += 1
        elif (ServoID == 23 or ServoID == 24):
            Tail_limbCount += 1

        if (FR_limbCount == 4):
            FR_Limb = [ServoObjDict[1],ServoObjDict[2],ServoObjDict[3],ServoObjDict[4]]
            FR_Leg = Leg(1,FR_Limb)
            TheoLimbList.append(FR_Leg)
            TheoLimbDict[1] = FR_Leg
            print("Front Right Limb is Digitally Assembled.")
        if (FL_limbCount == 4):
            FL_Limb = [ServoObjDict[5],ServoObjDict[6],ServoObjDict[7],ServoObjDict[8]]
            FL_Leg = Leg(2,FL_Limb)
            TheoLimbList.append(FL_Leg)
            TheoLimbDict[2] = FL_Leg
            print("Front Left Limb is Digitally Assembled.")
        if (BR_limbCount == 4):
            BR_Limb = [ServoObjDict[9],ServoObjDict[10],ServoObjDict[11],ServoObjDict[12]]
            BR_Leg = Leg(3,BR_Limb)
            TheoLimbList.append(BR_Leg)
            TheoLimbDict[3] = BR_Leg
            print("Back Right Limb is Digitally Assembled.")
        if (BL_limbCount == 4):
            BL_Limb = [ServoObjDict[13],ServoObjDict[14],ServoObjDict[15],ServoObjDict[16]]
            BL_Leg = Leg(4,BL_Limb)
            TheoLimbList.append(BL_Leg)
            TheoLimbDict[4] = BL_Leg
            print("Back Left Limb is Digitally Assembled.")
        if (Neck_limbCount == 2):
            Neck_Limb = [ServoObjDict[17],ServoObjDict[18]]
            NeckStructure = Neck(5,Neck_Limb)
            TheoLimbList.append(NeckStructure)
            TheoLimbDict[5] = NeckStructure
            print("Neck Limb is Digitally Assembled.")
        if (Spine_limbCount == 4):
            Spine_Limb = [ServoObjDict[19],ServoObjDict[20],ServoObjDict[21],ServoObjDict[22]]
            SpineStructure = Spine(6,Spine_Limb)
            TheoLimbList.append(SpineStructure)
            TheoLimbDict[6] = SpineStructure
            print("Spine Limb is Digitally Assembled.")
        if (Tail_limbCount == 2):
            Tail_Limb = [ServoObjDict[23],ServoObjDict[24]]
            TailStructure = Tail(7,Tail_Limb)
            TheoLimbList.append(TailStructure)
            TheoLimbDict[7] = TailStructure
            print("Tail Limb is Digitally Assembled.")

    TheoBody = Body(TheoLimbList)

    return ServoObjList, ServoObjDict, TheoLimbList, TheoLimbDict, TheoBody

def RunThreads(ObjToMove,portHandler,packetHandler):
    global stopVal
    t1 = Thread(target=ObjToMove.ContinuousMove,args=(portHandler,packetHandler))
    t2 = Thread(target=DetectStopInput)
    #thread_running = True
    t1.start()
    t2.start()
    t2.join()
    #thread_running = False
    stopVal = 0

def PulsePin(PinNum):
    GPIO.output(PinNum, 1)
    time.sleep(PinPulseTime)
    GPIO.output(PinNum, 0)
