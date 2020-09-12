import os
import ControlTable

def InitialSetup():
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


def SetServoVelocity(ServoID,ServoVel):
    #Set velocity limit 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_H)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Velocity limit set to: %03d" %(ServoID, VELOCITY_LIMIT_H))

def MoveSingleServo(ServoID,DesPos,DesSpeed,DisplayData):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_PRO_GOAL_POSITION, DesPos)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Goal Position set to: %03d" %(ServoID, DesPos))



def MoveSingleLimb(DesiredLimb,StaticOrContinuous):
    # Initialize GroupSyncWrite instance
    groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    # Initialize GroupSyncWrite instance
    groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_VELOCITY_LIMIT, LEN_VELOCITY_LIMIT)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if (DesiredLimb == 1):
        limb = F_R_ARM
        ServoPos1 = RotatePositionArray(FL_TOT_R_1,1,len(FL_TOT_R_1))
        ServoPos2 = RotatePositionArray(FL_TOT_R_2,1,len(FL_TOT_R_2))
        ServoPos3 = RotatePositionArray(FL_TOT_R_3,1,len(FL_TOT_R_3))
        ServoPos4 = RotatePositionArray(FL_TOT_R_4,1,len(FL_TOT_R_4))
        ServoVel1 =
        ServoVel2 =
        ServoVel3 =
        ServoVel4 =
    elif (DesiredLimb == 2):
        limb = F_L_ARM
        ServoPos1 = RotatePositionArray(FL_TOT_L_1,9,len(FL_TOT_L_1))
        ServoPos2 = RotatePositionArray(FL_TOT_L_2,9,len(FL_TOT_L_2))
        ServoPos3 = RotatePositionArray(FL_TOT_L_3,9,len(FL_TOT_L_3))
        ServoPos4 = RotatePositionArray(FL_TOT_L_4,9,len(FL_TOT_L_4))
        ServoVel1 =
        ServoVel2 =
        ServoVel3 =
        ServoVel4 =
    elif (DesiredLimb == 3):
        limb = B_R_ARM
        ServoPos1 = RotatePositionArray(HL_TOT_R_1,8,len(HL_TOT_R_1))
        ServoPos2 = RotatePositionArray(HL_TOT_R_2,8,len(HL_TOT_R_2))
        ServoPos3 = RotatePositionArray(HL_TOT_R_3,8,len(HL_TOT_R_3))
        ServoPos4 = RotatePositionArray(HL_TOT_R_4,8,len(HL_TOT_R_4))
        ServoVel1 =
        ServoVel2 =
        ServoVel3 =
        ServoVel4 =
    elif (DesiredLimb == 4):
        limb = B_L_ARM
        ServoPos1 = RotatePositionArray(HL_TOT_L_1,1,len(HL_TOT_L_1))
        ServoPos2 = RotatePositionArray(HL_TOT_L_2,1,len(HL_TOT_L_2))
        ServoPos3 = RotatePositionArray(HL_TOT_L_3,1,len(HL_TOT_L_3))
        ServoPos4 = RotatePositionArray(HL_TOT_L_4,1,len(HL_TOT_L_4))
        ServoVel1 =
        ServoVel2 =
        ServoVel3 =
        ServoVel4 =
    elif (DesiredLimb == 5):
        limb - NECK
    elif (DesiredLimb == 6):
        limb = SPINE
    elif (DesiredLimb == 7):
        limb = TAIL

    TurnOffOnTorque(TORQUE_ENABLE,1,limb[0],limb[-1])

    if (DesiredLimb == 1 or DesiredLimb == 2 or DesiredLimb == 3 or DesiredLimb == 4):
        index = 0
        # Add parameter storage for Dynamixel#1 present position value
        while 1:
            dxl_addparam_result = groupSyncRead.addParam(limb[0])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % limb[0])
                quit()

            # Add parameter storage for Dynamixel#2 present position value
            dxl_addparam_result = groupSyncRead.addParam(limb[1])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % limb[1])
                quit()

            dxl_addparam_result = groupSyncRead.addParam(limb[2])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % limb[2])
                quit()

            # Add parameter storage for Dynamixel#2 present position value
            dxl_addparam_result = groupSyncRead.addParam(limb[3])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % limb[3])
                quit()

            index = 0
            while 1:
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
                
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

                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWritePOS.addParam(limb[2],goal_position_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % limb[2])
                    quit()

                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
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

    elif (DesiredLimb == 5)

    elif (DesiredLimb == 6)

    elif (DesiredLimb == 7)

    
        


    #Turn torque off when done
    TurnOffOnTorque(TORQUE_DISABLE,1,limb[0],limb[-1])

    

def MoveRobotHome():
    pass

def MoveEntireBody():
    pass


def ReturnRelevantData():
    pass



def DetermineSpeeds(StrideTime,positionsFile):
    import numpy as np
    import pandas as pd
    import math
    import time
    import copy as cp
    import sys

    df = pd.read_csv(positionsFile)
    df = df.values[:][:]
    cf = cp.copy(df)
    h_stance = 4.892994
    h_swing = 3.347006
    f_stance = 5.211718
    f_swing = 3.028282
    h_st_per = h_stance / 8.24
    h_sw_per = h_swing / 8.24
    f_st_per = f_stance / 8.24
    f_sw_per = f_swing / 8.24
    b = cf.shape
    cTotal = b[0]
    cLength = cTotal//2
    cWidth = b[1]
    percents = np.linspace(1,cLength-1,cLength-1)
    joints = list(range(1,cWidth+1))
    percents = percents.astype(int).tolist()
    distTravel = np.zeros((11,16))
    for i in percents:
        for j in joints:
            if (j == 1 or j == 2 or j == 3 or j == 4 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*f_st_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 5 or j == 6 or j == 7 or j == 8): 
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*f_sw_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 9 or j == 10 or j == 11 or j == 12 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*h_st_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 13 or j == 14 or j == 15 or j == 16 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*h_sw_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
    cf = np.round(cf)
    joints = list(range(0,cWidth))
    for j in joints:
        if (j == 0 or j == 1 or j == 2 or j == 3):
            R1 = abs(df[-1][j] - df[0][j+4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * f_st_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 4 or j == 5 or j == 6 or j == 7):
            R1 = abs(df[-1][j] - df[0][j-4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * f_sw_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 8 or j == 9 or j == 10 or j == 11):
            R1 = abs(df[-1][j] - df[0][j+4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * h_st_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 12 or j == 13 or j == 14 or j == 15):
            R1 = abs(df[-1][j] - df[0][j-4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * h_sw_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
    cf[cf==0]=1
    rows = cf.shape[0]
    cols = cf.shape[1]
    for x in range(0, rows):
        for y in range(0, cols):
            if (cf[x][y] > 1023):
                cf[x][y] = 1023
    efa = distTravel[:,(0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11)]
    efb = distTravel[:,(4,5,6,7,4,5,6,7,12,13,14,15,12,13,14,15)]
    distances = np.concatenate((efa,efb),axis=0)
    for p in [0,1,2,3]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-1)
        distances[:,p] = efc1
    for p in [4,5,6,7]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-9)
        distances[:,p] = efc1
    for p in [8,9,10,11]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-8)
        distances[:,p] = efc1
    cfa = cf[:,(0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11)]
    cfb = cf[:,(4,5,6,7,4,5,6,7,12,13,14,15,12,13,14,15)]
    cfc = np.concatenate((cfa,cfb),axis=0)
    for q in [0,1,2,3]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-1)
        cfc[:,q] = cfc1       
    for q in [4,5,6,7]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-9)
        cfc[:,q] = cfc1
    for q in [8,9,10,11]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-8)
        cfc[:,q] = cfc1
    speeds = np.multiply(cfc,.114)
    distances[distances==0]=.0000001
    ansFinal = np.divide(distances,speeds)
    slowest = []
    for k in range(0,22):
        movement = ansFinal[k][:]
        temp1 = movement.max()
        movement = movement.tolist()
        temp2 = movement.index(temp1)
        slowest.append(temp2)
    slowestServos = [x + 1 for x in slowest]





def ExtractInfo():
    pass


def ChangeSpecificTrait():
    pass



def CleanUp():
    pass
