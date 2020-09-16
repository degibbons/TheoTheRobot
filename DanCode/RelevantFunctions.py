import os
from ControlTable import *
from dynamixel_sdk import *
import numpy as np
import copy as cp

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

def TurnOffOnTorque(OnOrOff,AllOrOne,StartServo,EndServo):
    if (AllOrOne == 1):
        index = range(StartServo,EndServo + 1)
    elif (AllOrOne == 0):
        index = [StartServo]
    else:
        pass

    for ID in index:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_PRO_TORQUE_ENABLE, OnOrOff)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            if (OnOrOff == 1):
                print("Dynamixel#%d torque on" % ID)
            elif (OnOrOff == 0):
                print("Dynamixel#%d torque off" % ID)

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

def SetServoTraits(ServoID):
    #Set drive mode to velocity based
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ServoID, ADDR_DRIVE_MODE, DRIVE_MODE_VEL_BASED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Drive mode set to: %03d" %(ServoID, DRIVE_MODE_VEL_BASED))

    #Set operating mode to joint/position control mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ServoID, ADDR_OPERATING_MODE, OPERATING_JOINT_POSITION_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Operating mode set to: %03d" %(ServoID, OPERATING_JOINT_POSITION_MODE))

    #Set acceleration limit 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_ACCELERATION_LIMIT, ACCELERATION_LIMIT_M)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Acceleration limit set to: %03d" %(ServoID, ACCELERATION_LIMIT_M))

    #Set max position limit
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Max position limit set to: %03d" %(ServoID, MAX_POSITION_LIMIT))

    #Set min position limit
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Min position limit set to: %03d" %(ServoID, MIN_POSITION_LIMIT))

    #Set moving threshold
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD_ACCURACY_H)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Moving accuracy set to high: %03d" %(ServoID, MOVING_THRESHOLD_ACCURACY_H))

def SetSingleServoVelocity(ServoID,ServoVel):
    #Set velocity limit 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_H)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Velocity limit set to: %03d" %(ServoID, VELOCITY_LIMIT_H))

def MoveSingleServo(ServoID,DesPos):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_PRO_GOAL_POSITION, DesPos)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Goal Position set to: %03d" %(ServoID, DesPos))

def MoveSingleLimb(DesiredLimb,PositionMatrix,SpeedMatrix,indexIn):
    # Initialize GroupSyncWrite instance
    groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    # Initialize GroupSyncWrite instance
    groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_VELOCITY_LIMIT, LEN_VELOCITY_LIMIT)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if (DesiredLimb == 1):
        limb = F_R_ARM

        ServoPos1 = PositionMatrix[indexIn,0]
        ServoPos2 = PositionMatrix[indexIn,1]
        ServoPos3 = PositionMatrix[indexIn,2]
        ServoPos4 = PositionMatrix[indexIn,3]

        ServoVel1 = SpeedMatrix[indexIn,0]
        ServoVel2 = SpeedMatrix[indexIn,1]
        ServoVel3 = SpeedMatrix[indexIn,2]
        ServoVel4 = SpeedMatrix[indexIn,3]

    elif (DesiredLimb == 2):
        limb = F_L_ARM

        ServoPos1 = PositionMatrix[indexIn,4]
        ServoPos2 = PositionMatrix[indexIn,5]
        ServoPos3 = PositionMatrix[indexIn,6]
        ServoPos4 = PositionMatrix[indexIn,7]

        ServoVel1 = SpeedMatrix[indexIn,4]
        ServoVel2 = SpeedMatrix[indexIn,5]
        ServoVel3 = SpeedMatrix[indexIn,6]
        ServoVel4 = SpeedMatrix[indexIn,7]
                
    elif (DesiredLimb == 3):
        limb = B_R_ARM

        ServoPos1 = PositionMatrix[indexIn,8]
        ServoPos2 = PositionMatrix[indexIn,9]
        ServoPos3 = PositionMatrix[indexIn,10]
        ServoPos4 = PositionMatrix[indexIn,11]

        ServoVel1 = SpeedMatrix[indexIn,8]
        ServoVel2 = SpeedMatrix[indexIn,9]
        ServoVel3 = SpeedMatrix[indexIn,10]
        ServoVel4 = SpeedMatrix[indexIn,11]
        
    elif (DesiredLimb == 4):
        limb = B_L_ARM

        ServoPos1 = PositionMatrix[indexIn,12]
        ServoPos2 = PositionMatrix[indexIn,13]
        ServoPos3 = PositionMatrix[indexIn,14]
        ServoPos4 = PositionMatrix[indexIn,15]
        
        ServoVel1 = SpeedMatrix[indexIn,12]
        ServoVel2 = SpeedMatrix[indexIn,13]
        ServoVel3 = SpeedMatrix[indexIn,14]
        ServoVel4 = SpeedMatrix[indexIn,15]

    elif (DesiredLimb == 5):
        limb - NECK

    elif (DesiredLimb == 6):
        limb = SPINE

    elif (DesiredLimb == 7):
        limb = TAIL

    if (DesiredLimb == 1 or DesiredLimb == 2 or DesiredLimb == 3 or DesiredLimb == 4):
        index = 0
        # Add parameter storage for Dynamixel#1 present position value
        
        dxl_addparam_result = groupSyncRead.addParam(limb[0])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[0])
            quit()

        # Add parameter storage for Dynamixel#2 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[1])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[1])
            quit()
        # Add parameter storage for Dynamixel#3 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[2])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[2])
            quit()

        # Add parameter storage for Dynamixel#4 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[3])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[3])
            quit()

        goal_velocity_1 = [DXL_LOBYTE(DXL_LOWORD(ServoVel1[index])), DXL_HIBYTE(DXL_LOWORD(ServoVel1[index])), DXL_LOBYTE(DXL_HIWORD(ServoVel1[index])), DXL_HIBYTE(DXL_HIWORD(ServoVel1[index]))]
        goal_velocity_2 = [DXL_LOBYTE(DXL_LOWORD(ServoVel2[index])), DXL_HIBYTE(DXL_LOWORD(ServoVel2[index])), DXL_LOBYTE(DXL_HIWORD(ServoVel2[index])), DXL_HIBYTE(DXL_HIWORD(ServoVel2[index]))]
        goal_velocity_3 = [DXL_LOBYTE(DXL_LOWORD(ServoVel3[index])), DXL_HIBYTE(DXL_LOWORD(ServoVel3[index])), DXL_LOBYTE(DXL_HIWORD(ServoVel3[index])), DXL_HIBYTE(DXL_HIWORD(ServoVel3[index]))]
        goal_velocity_4 = [DXL_LOBYTE(DXL_LOWORD(ServoVel4[index])), DXL_HIBYTE(DXL_LOWORD(ServoVel4[index])), DXL_LOBYTE(DXL_HIWORD(ServoVel4[index])), DXL_HIBYTE(DXL_HIWORD(ServoVel4[index]))]

        # Add Dynamixel#1 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[0],goal_velocity_1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[0])
            quit()

        # Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[1],goal_velocity_2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[1])
            quit()

        # Add Dynamixel#3 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[2],goal_velocity_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[2])
            quit()

        # Add Dynamixel#4 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[3],goal_velocity_4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[3])
            quit()

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(ServoPos1[index])), DXL_HIBYTE(DXL_LOWORD(ServoPos1[index])), DXL_LOBYTE(DXL_HIWORD(ServoPos1[index])), DXL_HIBYTE(DXL_HIWORD(ServoPos1[index]))]
        goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(ServoPos2[index])), DXL_HIBYTE(DXL_LOWORD(ServoPos2[index])), DXL_LOBYTE(DXL_HIWORD(ServoPos2[index])), DXL_HIBYTE(DXL_HIWORD(ServoPos2[index]))]
        goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(ServoPos3[index])), DXL_HIBYTE(DXL_LOWORD(ServoPos3[index])), DXL_LOBYTE(DXL_HIWORD(ServoPos3[index])), DXL_HIBYTE(DXL_HIWORD(ServoPos3[index]))]
        goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(ServoPos4[index])), DXL_HIBYTE(DXL_LOWORD(ServoPos4[index])), DXL_LOBYTE(DXL_HIWORD(ServoPos4[index])), DXL_HIBYTE(DXL_HIWORD(ServoPos4[index]))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[0],goal_position_1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[0])
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[1],goal_position_2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[1])
            quit()

        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[2],goal_position_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[2])
            quit()

        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[3],goal_position_4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[3])
            quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[0])
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[1], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[1])
            quit()

        # Check if groupsyncread data of Dynamixel#3 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[2], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[2])
            quit()

        # Check if groupsyncread data of Dynamixel#4 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[3], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[3])
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(limb[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(limb[1], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#3 present position value
        dxl3_present_position = groupSyncRead.getData(limb[2], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#4 present position value
        dxl4_present_position = groupSyncRead.getData(limb[3], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[0], ServoPos1, dxl1_present_position, limb[1], ServoPos2, dxl2_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[2], ServoPos3, dxl3_present_position, limb[3], ServoPos4, dxl4_present_position))

        # Clear syncread parameter storage
        groupSyncRead.clearParam()

    elif (DesiredLimb == 5):
        # Neck
        pass
    elif (DesiredLimb == 6):
        # Spine
        pass
    elif (DesiredLimb == 7):
        # Tail
        pass
    
        


    #Turn torque off when 
    # PUT THIS IN MASTER SCRIPT
    TurnOffOnTorque(TORQUE_DISABLE,1,limb[0],limb[-1])

def MoveLimbHome(DesiredLimb,PositionMatrix,SpeedMatrix):
    MoveSingleLimb(DesiredLimb,PositionMatrix,SpeedMatrix,0)

def MoveLimbsHome(PositionMatrix,SpeedMatrix):
    MoveLimbHome(1,PositionMatrix,SpeedMatrix,0)
    MoveLimbHome(2,PositionMatrix,SpeedMatrix,0)
    MoveLimbHome(3,PositionMatrix,SpeedMatrix,0)
    MoveLimbHome(4,PositionMatrix,SpeedMatrix,0)

def StraightenSpine():
    MoveSingleLimb(6,STRAIGHT_SPINE_ARRAY,STRAIGHT_SPEED_ARRAY,0)

def MoveEntireBody(PositionMatrix,SpeedMatrix):
    index = 1
    while 1:
        MoveLimbHome(1,PositionMatrix,SpeedMatrix,index)
        MoveLimbHome(2,PositionMatrix,SpeedMatrix,index)
        MoveLimbHome(3,PositionMatrix,SpeedMatrix,index)
        MoveLimbHome(4,PositionMatrix,SpeedMatrix,index)
        index += 1
        if (index > 21):
            index = 0
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

def DetermineSpeeds(StrideTime,positionsFile):
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
    cLength = b[0]
    cWidth = b[1]
    percents = np.linspace(1,cLength-1,cLength-1)
    joints = list(range(1,cWidth+1))
    percents = percents.astype(int).tolist()
    distTravel = np.zeros((11,16))
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

    return FL_VEL, HL_VEL 

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
    print("\nWhat would you like to do?\n")
    print("---------MENU---------")
    print("1: Move a single servo")
    print("2: Move a specific limb")
    print("3: Move the entire robot")
    print("4: Other")
    print("5: Exit\n")

def ReturnRelevantData(DesiredData,DesiredServo):
    pass

def WriteDataToDoc():
    pass

def ChangeSpecificTrait():
    pass

def CleanUp(number_of_servos_connected):
    TurnOffOnTorque(TORQUE_OFF,1,1,number_of_servos_connected)
    print("Shutting down system.\n")
    print("Thank you for using Theo!")
    
