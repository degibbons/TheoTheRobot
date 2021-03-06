## NEEDS COMMENTS

#Also put default values for functions that need them

import os
from ControlTable import *
from dynamixel_sdk import *
import numpy as np
import copy as cp
import time
from curtsies import Input
from threading import Thread
import RPi.GPIO as GPIO

stopVal = 0

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

def TurnOffOnTorque(OnOrOff,AllOrOne,StartServo,EndServo,portHandler,packetHandler):
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

def AskAboutFile():
    fileName = "Example.txt"
    recordInput = input("Do you want to Record the Servo Data to a file? [Y/N]: ")
    if (recordInput.lower() == "y"):
        doesFileExist = input("Do you want to add to an existing file[0] or start a new one[1]?: ")
        # Need to check if file does in fact exist if 0  was input !!
        if (doesFileExist == 1):
            filePermission = 'w'
            print("Your data file will be saved in the Records sub-folder.\n")
            DesiredFileName = input("What do you want the file to be named?(will be followed by .txt): ")
            DesiredFileName = DesiredFileName + '.txt'
            fileName = DesiredFileName
        elif (doesFileExist == 0):
            # Need way to easily access name of File that already exists
            # This will be added when Tkinter is introduced for GUI
            filePermission = 'a'
            FileExist = 1
            #fileName = PREVIOUSLY CREATED FILE
        else:
            pass
    elif (recordInput.lower() == "n"):
        filePermission = 'n'
    else:
        pass
    return filePermission, fileName

def WriteDataToDoc(inData,FileName,FileExist):
    # If First run, file exist = 0
    # Set to 1 if already exists or back to 0 if making a new one
    if (FileExist == 0):
        f = open(FileName,"w+")
    elif (FileExist == 1):
        f = open(FileName,"a+")
    else:
        pass
    f.write(inData)
    f.close()

def SetServoTraits(ServoID,portHandler,packetHandler):
    #Set drive mode to velocity based
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ServoID, ADDR_DRIVE_MODE, DRIVE_MODE_VEL_BASED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Drive mode set to: %03d" %(ServoID, DRIVE_MODE_VEL_BASED))

    time.sleep(PreferedDelay)

    #Set operating mode to joint/position control mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ServoID, ADDR_OPERATING_MODE, OPERATING_JOINT_POSITION_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Operating mode set to: %03d" %(ServoID, OPERATING_JOINT_POSITION_MODE))

    time.sleep(PreferedDelay)

    #Set acceleration limit 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_ACCELERATION_LIMIT, ACCELERATION_LIMIT_M)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Acceleration limit set to: %03d" %(ServoID, ACCELERATION_LIMIT_M))

    time.sleep(PreferedDelay)

    #Set max position limit
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Max position limit set to: %03d" %(ServoID, MAX_POSITION_LIMIT))

    time.sleep(PreferedDelay)

    #Set min position limit
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Min position limit set to: %03d" %(ServoID, MIN_POSITION_LIMIT))

    time.sleep(PreferedDelay)

    #Set moving threshold
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD_ACCURACY_H)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Moving accuracy set to high: %03d" %(ServoID, MOVING_THRESHOLD_ACCURACY_H))

    time.sleep(PreferedDelay)

def SetSingleServoVelocity(ServoID,ServoVel,portHandler,packetHandler):
    #Set velocity limit 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_PROFILE_VELOCITY, VELOCITY_LIMIT_H)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Velocity limit set to: %03d" %(ServoID, VELOCITY_LIMIT_H))

def MoveSingleServo(ServoID,DesPos,portHandler,packetHandler):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ServoID, ADDR_PRO_GOAL_POSITION, DesPos)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Goal Position set to: %03d" %(ServoID, DesPos))

def MoveSingleLimb(DesiredLimb,PositionMatrix,SpeedMatrix,indexIn, StrideIndex, portHandler,packetHandler,filePermission,fileName,*args):
    # Initialize GroupSyncWrite instance
    groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    # Initialize GroupSyncWrite instance
    groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if (DesiredLimb == 1):
        limb = F_R_ARM
        limbNum = 1
        ServoPos1 = PositionMatrix[indexIn,0]
        ServoPos2 = PositionMatrix[indexIn,1]
        ServoPos3 = PositionMatrix[indexIn,2]
        ServoPos4 = PositionMatrix[indexIn,3]

        ServoVel1 = int(SpeedMatrix[indexIn,0])
        ServoVel2 = int(SpeedMatrix[indexIn,1])
        ServoVel3 = int(SpeedMatrix[indexIn,2])
        ServoVel4 = int(SpeedMatrix[indexIn,3])

    elif (DesiredLimb == 2):
        limb = F_L_ARM
        limbNum = 2
        ServoPos1 = PositionMatrix[indexIn,4]
        ServoPos2 = PositionMatrix[indexIn,5]
        ServoPos3 = PositionMatrix[indexIn,6]
        ServoPos4 = PositionMatrix[indexIn,7]

        ServoVel1 = int(SpeedMatrix[indexIn,4])
        ServoVel2 = int(SpeedMatrix[indexIn,5])
        ServoVel3 = int(SpeedMatrix[indexIn,6])
        ServoVel4 = int(SpeedMatrix[indexIn,7])
                
    elif (DesiredLimb == 3):
        limb = B_R_ARM
        limbNum = 3
        ServoPos1 = PositionMatrix[indexIn,8]
        ServoPos2 = PositionMatrix[indexIn,9]
        ServoPos3 = PositionMatrix[indexIn,10]
        ServoPos4 = PositionMatrix[indexIn,11]

        ServoVel1 = int(SpeedMatrix[indexIn,8])
        ServoVel2 = int(SpeedMatrix[indexIn,9])
        ServoVel3 = int(SpeedMatrix[indexIn,10])
        ServoVel4 = int(SpeedMatrix[indexIn,11])
        
    elif (DesiredLimb == 4):
        limb = B_L_ARM
        limbNum = 4
        ServoPos1 = PositionMatrix[indexIn,12]
        ServoPos2 = PositionMatrix[indexIn,13]
        ServoPos3 = PositionMatrix[indexIn,14]
        ServoPos4 = PositionMatrix[indexIn,15]
        
        ServoVel1 = int(SpeedMatrix[indexIn,12])
        ServoVel2 = int(SpeedMatrix[indexIn,13])
        ServoVel3 = int(SpeedMatrix[indexIn,14])
        ServoVel4 = int(SpeedMatrix[indexIn,15])

    elif (DesiredLimb == 5):
        limb - NECK

    elif (DesiredLimb == 6):
        limb = SPINE

    elif (DesiredLimb == 7):
        limb = TAIL

    if (DesiredLimb == 1 or DesiredLimb == 2 or DesiredLimb == 3 or DesiredLimb == 4):
        # Add parameter storage for Dynamixel#1 present position value
        
        dxl_addparam_result = groupSyncRead.addParam(limb[0])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[0])
            quit()

        #time.sleep(PreferedDelay)

        # Add parameter storage for Dynamixel#2 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[1])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[1])
            quit()

        #time.sleep(PreferedDelay)

        # Add parameter storage for Dynamixel#3 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[2])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[2])
            quit()

        #time.sleep(PreferedDelay)

        # Add parameter storage for Dynamixel#4 present position value
        dxl_addparam_result = groupSyncRead.addParam(limb[3])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % limb[3])
            quit()

        #time.sleep(PreferedDelay)

        #goal_velocity_1 = [DXL_LOBYTE(DXL_LOWORD(ServoVel1)), DXL_HIBYTE(DXL_LOWORD(ServoVel1)), DXL_LOBYTE(DXL_HIWORD(ServoVel1)), DXL_HIBYTE(DXL_HIWORD(ServoVel1))]
        #goal_velocity_2 = [DXL_LOBYTE(DXL_LOWORD(ServoVel2)), DXL_HIBYTE(DXL_LOWORD(ServoVel2)), DXL_LOBYTE(DXL_HIWORD(ServoVel2)), DXL_HIBYTE(DXL_HIWORD(ServoVel2))]
        #goal_velocity_3 = [DXL_LOBYTE(DXL_LOWORD(ServoVel3)), DXL_HIBYTE(DXL_LOWORD(ServoVel3)), DXL_LOBYTE(DXL_HIWORD(ServoVel3)), DXL_HIBYTE(DXL_HIWORD(ServoVel3))]
        #goal_velocity_4 = [DXL_LOBYTE(DXL_LOWORD(ServoVel4)), DXL_HIBYTE(DXL_LOWORD(ServoVel4)), DXL_LOBYTE(DXL_HIWORD(ServoVel4)), DXL_HIBYTE(DXL_HIWORD(ServoVel4))]
        goal_velocity_1 = FormatSendData(ServoVel1)
        goal_velocity_2 = FormatSendData(ServoVel2)
        goal_velocity_3 = FormatSendData(ServoVel3)
        goal_velocity_4 = FormatSendData(ServoVel4)

        # Add Dynamixel#1 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[0],goal_velocity_1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[0])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[1],goal_velocity_2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[1])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#3 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[2],goal_velocity_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[2])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#4 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVEL.addParam(limb[3],goal_velocity_4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[3])
            quit()
        
        #time.sleep(PreferedDelay)

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        #goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(ServoPos1)), DXL_HIBYTE(DXL_LOWORD(ServoPos1)), DXL_LOBYTE(DXL_HIWORD(ServoPos1)), DXL_HIBYTE(DXL_HIWORD(ServoPos1))]
        #goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(ServoPos2)), DXL_HIBYTE(DXL_LOWORD(ServoPos2)), DXL_LOBYTE(DXL_HIWORD(ServoPos2)), DXL_HIBYTE(DXL_HIWORD(ServoPos2))]
        #goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(ServoPos3)), DXL_HIBYTE(DXL_LOWORD(ServoPos3)), DXL_LOBYTE(DXL_HIWORD(ServoPos3)), DXL_HIBYTE(DXL_HIWORD(ServoPos3))]
        #goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(ServoPos4)), DXL_HIBYTE(DXL_LOWORD(ServoPos4)), DXL_LOBYTE(DXL_HIWORD(ServoPos4)), DXL_HIBYTE(DXL_HIWORD(ServoPos4))]
        goal_position_1 = FormatSendData(ServoPos1)
        goal_position_2 = FormatSendData(ServoPos2)
        goal_position_3 = FormatSendData(ServoPos3)
        goal_position_4 = FormatSendData(ServoPos4)

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[0],goal_position_1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[0])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[1],goal_position_2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[1])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[2],goal_position_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[2])
            quit()

        #time.sleep(PreferedDelay)

        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePOS.addParam(limb[3],goal_position_4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % limb[3])
            quit()

        #time.sleep(PreferedDelay)
        
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

        #time.sleep(PreferedDelay)

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[0])
            quit()

        #time.sleep(PreferedDelay)

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[1], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[1])
            quit()

        #time.sleep(PreferedDelay)

        # Check if groupsyncread data of Dynamixel#3 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[2], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[2])
            quit()

        #time.sleep(PreferedDelay)

        # Check if groupsyncread data of Dynamixel#4 is available
        dxl_getdata_result = groupSyncRead.isAvailable(limb[3], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % limb[3])
            quit()  

        #time.sleep(PreferedDelay)

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(limb[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        #time.sleep(PreferedDelay)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(limb[1], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        #time.sleep(PreferedDelay)

        # Get Dynamixel#3 present position value
        dxl3_present_position = groupSyncRead.getData(limb[2], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        #time.sleep(PreferedDelay)

        # Get Dynamixel#4 present position value
        dxl4_present_position = groupSyncRead.getData(limb[3], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        #time.sleep(PreferedDelay)

        GetPos12 = "[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[0], ServoPos1, dxl1_present_position, limb[1], ServoPos2, dxl2_present_position)
        GetPos34 = "[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[2], ServoPos3, dxl3_present_position, limb[3], ServoPos4, dxl4_present_position)
        GetVel12 = "[ID:%03d] Velocity Given:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[0], ServoVel1, limb[1], ServoVel2, dxl2_present_position)
        GetVel34 = "[ID:%03d] Velocity Given:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (limb[2], ServoVel3, limb[3], ServoVel4, dxl4_present_position)
        GetMoveIndex = f"Movement #{StrideIndex}"

        print("\n")
        print("==========================================================================================")
        print(GetMoveIndex)
        print(GetPos12)
        print(GetVel12)
        print(GetPos34)
        print(GetVel34)
        print("==========================================================================================")

        if (filePermission == 'w'):
            WriteDataToDoc(GetPos12,fileName,0)
            WriteDataToDoc('\n',fileName,0)
            WriteDataToDoc(GetPos34,fileName,0)
        elif (filePermission == 'a'):
            WriteDataToDoc(GetPos12,fileName,1)
            WriteDataToDoc('\n',fileName,1)
            WriteDataToDoc(GetPos34,fileName,1)
        elif (filePermission == 'n'):
            pass

        # Clear syncread parameter storage
        groupSyncRead.clearParam()

        # if limbNum == 1:
        #     StartIndex =  
        #     EndIndex = 
        #     HomeIndex = 
        #     FirstMoveIndex = 
        #     St2SwIndex = 
        #     Sw2StIndex = 
        # elif limbNum == 2:
        #     StartIndex = 
        #     EndIndex = 
        #     HomeIndex = 
        #     FirstMoveIndex = 
        #     St2SwIndex = 
        #     Sw2StIndex = 
        # elif limbNumb == 3:
        #     StartIndex = 
        #     EndIndex = 
        #     HomeIndex = 
        #     FirstMoveIndex = 
        #     St2SwIndex = 
        #     Sw2StIndex = 
        # elif limbNumb == 4:
        #     StartIndex = 
        #     EndIndex = 
        #     HomeIndex = 
        #     FirstMoveIndex = 
        #     St2SwIndex = 
        #    Sw2StIndex = 
            

        if indexIn == 0:
            HomePosCheck = 1
            FirstMoveCheck = 0
            STorSW = 0
            PhaseIndex = 0
        else:
            HomePosCheck = 0

            if indexIn == 1:
                FirstMoveCheck = 1
            else:
                FirstMoveCheck = 0

            if indexIn >= 1 and indexIn <=10:
                STorSW = 1
            elif indexIn >= 12 and indexIn <= 21:
                STorSW = -1
            elif indexIn == 0 or indexIn == 11:
                STorSW = 0

            if indexIn > 10:
                PhaseIndex = indexIn - 11
            else:
                PhaseIndex = indexIn

        
        
        # ^^^ THESE WILL CHANGE BECAUSE THE POSITIONS BECOME SHIFTED 

        # MAKE the numbers variables that change with respect to what limb is being checked
        

        if args:
            BeginTime = args[0]
            PhaseTime = args[1]
            StrideTime = args[2]

        
        TimeMarker2 = time.perf_counter()
        TimeMarker3 = time.perf_counter()
        TimeMarker4 = time.perf_counter()
        TimeMarkerSlowest = time.perf_counter
        
        # PhaseTime = BeginTime - 
        # StrideTime = BeginTime - 
        # TotTime = BeginTime - 


        # Limb Number, Servo Number, Goal Position, Present Position, Speed Given to get to Goal Position, 
        # Is this Home Position?, Is This the First Position Moved after home position?, What Number Stride is this, 
        # Is This Stance or Swing?, What Index in Stance or Swing is this?, What Time Value in Stance or Swing Is this?[Time from begining of stance to now],
        #  What Index in the whole Stride is this[Time from beginning of Stride to now]?, What time value within the whole stride is this?,
        # How long did this one movement take?, What Time overall is this?
        DataList1 = [limbNum, 1, ServoPos1, dxl1_present_position, ServoVel1, HomePosCheck, FirstMoveCheck, StrideIndex, STorSW, PhaseIndex, PhaseTime, indexIn, StrideTime, MoveTime1, TotTime]
        DataList2 = [limbNum, 2, ServoPos2, dxl2_present_position, ServoVel2, HomePosCheck, FirstMoveCheck, StrideIndex, STorSW, PhaseIndex, PhaseTime, indexIn, StrideTime, MoveTime2, TotTime]
        DataList3 = [limbNum, 3, ServoPos3, dxl3_present_position, ServoVel3, HomePosCheck, FirstMoveCheck, StrideIndex, STorSW, PhaseIndex, PhaseTime, indexIn, StrideTime, MoveTime3, TotTime]
        DataList4 = [limbNum, 4, ServoPos4, dxl4_present_position, ServoVel4, HomePosCheck, FirstMoveCheck, StrideIndex, STorSW, PhaseIndex, PhaseTime, indexIn, StrideTime, MoveTime4, TotTime]

    elif (DesiredLimb == 5):
        # Neck
        pass
    elif (DesiredLimb == 6):
        # Spine
        pass
    elif (DesiredLimb == 7):
        # Tail
        pass
    
def LimbLoop(DesiredLimb,PositionMatrix,SpeedMatrix,index,portHandler,packetHandler,filePermission,fileName):
    global stopVal
    if (DesiredLimb == 1):
        limb = F_R_ARM
    elif (DesiredLimb == 2):
        limb = F_L_ARM
    elif (DesiredLimb == 3):
        limb = B_R_ARM
    elif (DesiredLimb == 4):
        limb = B_L_ARM
    elif (DesiredLimb == 5):
        limb = NECK
    elif (DesiredLimb == 6):
        limb = SPINE
    elif (DesiredLimb == 7):
        limb = TAIL
    else:
        print("Limb Number Not Recognized")
        return
        
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
 #   portHandlerCHECK = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
 #   packetHandlerCHECK = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncRead instace for Moving Status
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_MOVING, LEN_MOVING)
    
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
    StrideIndex = 1
    while 1:
        MoveSingleLimb(DesiredLimb,PositionMatrix,SpeedMatrix,index,StrideIndex,portHandler,packetHandler,filePermission,fileName)
        index += 1
        if (stopVal == 1):
            break
        if (index > 21):
            index = 0
            StrideIndex += 1
        key1 = 0
        key2 = 0
        key3 = 0
        key4 = 0
        while 1:
            # Syncread Moving Value
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Get Dynamixel#1 present Moving value
            dxl1_mov = groupSyncRead.getData(limb[0], ADDR_MOVING, LEN_MOVING)
            if (dxl1_mov == 0) and (key1 == 0):
                TimeMarker1 = time.perf_counter()
                key1 = 1


 #            time.sleep(PreferedDelay)

            # Get Dynamixel#2 Moving value
            dxl2_mov = groupSyncRead.getData(limb[1], ADDR_MOVING, LEN_MOVING)
            if (dxl2_mov == 0) and (key2 == 0):
                TimeMarker2 = time.perf_counter()
                key2 = 1
 #            time.sleep(PreferedDelay)

            # Get Dynamixel#3 Moving value
            dxl3_mov = groupSyncRead.getData(limb[2], ADDR_MOVING, LEN_MOVING)
            if (dxl3_mov == 0) and (key3 == 0):
                TimeMarker3 = time.perf_counter()
                key3 = 1
 #            time.sleep(PreferedDelay)

            # Get Dynamixel#4 Moving value
            dxl4_mov = groupSyncRead.getData(limb[3], ADDR_MOVING, LEN_MOVING)
            if (dxl4_mov == 0) and (key4 == 0):
                TimeMarker4 = time.perf_counter()
                key4 = 1

            if ((dxl1_mov == 1) or (dxl2_mov == 1) or (dxl3_mov == 1) or (dxl4_mov == 1)):
                pass
            else:
 #               groupSyncRead.clearParam()
                break
        # Write Data to Document HERE

def DetectStopInput():
    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            if (str(e) == '\x1b'):
                print("Stopping Servo Movement\n")
                global stopVal
                stopVal = 1
                break

def MoveLimbHome(DesiredLimb,PositionMatrix,SpeedMatrix,portHandler,packetHandler):
    MoveSingleLimb(DesiredLimb,PositionMatrix,SpeedMatrix,0,0,portHandler,packetHandler,'n','spacefiller')

def MoveLimbsHome(PositionMatrix,SpeedMatrix,portHandler,packetHandler):
    MoveLimbHome(1,PositionMatrix,SpeedMatrix,portHandler,packetHandler)
    MoveLimbHome(2,PositionMatrix,SpeedMatrix,portHandler,packetHandler)
    MoveLimbHome(3,PositionMatrix,SpeedMatrix,portHandler,packetHandler)
    MoveLimbHome(4,PositionMatrix,SpeedMatrix,portHandler,packetHandler)

def StraightenSpine(portHandler,packetHandler):
    MoveSingleLimb(6,STRAIGHT_SPINE_ARRAY,MoveHomeSpeedMatrix,0,0,portHandler,packetHandler,'n','spacefiller')

def MoveEntireBody(PositionMatrix,SpeedMatrix,portHandler,packetHandler,filePermission,fileName):
    global stopVal
    index = 1
    start = 0
    StrideIndex = 1
    while 1:
        if (start == 0):
            MoveLimbsHome(PositionMatrix,MoveHomeSpeedMatrix,portHandler,packetHandler)
            start = 1
            print("Press Enter to start.")
            while 1:
                if (getch() == chr(0x0D)):
                    break
        elif (start == 1):
            MoveSingleLimb(1,PositionMatrix,SpeedMatrix,index,StrideIndex,portHandler,packetHandler,filePermission,fileName)
            MoveSingleLimb(2,PositionMatrix,SpeedMatrix,index,StrideIndex,portHandler,packetHandler,filePermission,fileName)
            MoveSingleLimb(3,PositionMatrix,SpeedMatrix,index,StrideIndex,portHandler,packetHandler,filePermission,fileName)
            MoveSingleLimb(4,PositionMatrix,SpeedMatrix,index,StrideIndex,portHandler,packetHandler,filePermission,fileName)
        index += 1
        if (stopVal == 1):
            break
        if (index > 21):
            index = 0
            StrideIndex += 1
        
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
    print("5: Get Servo trait")
    print("6: Send Servo trait")
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

def ReadTraitData(DesiredData,DesiredServo):
    # Obtain from the desired servo the desired trait data

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

    DataAddr = DataAddrConversion(DesiredData)

    if (DesiredData == 1):
        ModelNumber, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Model Number: %03d" % (DesiredServo, ModelNumber))
    elif (DesiredData == 2):
        ModelInfo, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Model Information: %03d" % (DesiredServo, ModelInfo))
    elif (DesiredData == 3):
        FirmwareVer, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Firmware Version: %03d" % (DesiredServo, FirmwareVer))
    elif (DesiredData == 4):
        IDnum, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] ID Number: %03d" % (DesiredServo, IDnum))
    elif (DesiredData == 5):
        BaudRate, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Baud Rate: %03d" % (DesiredServo, BaudRate))
    elif (DesiredData == 6):
        RetDelayTime, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Return Delay Time: %03d" % (DesiredServo, RetDelayTime))
    elif (DesiredData == 7):
        DriveMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Drive Mode: %03d" % (DesiredServo, DriveMode))
    elif (DesiredData == 8):
        OperateMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Operating Mode: %03d" % (DesiredServo, OperateMode))
    elif (DesiredData == 9):
        ProtocType, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Protocol Type: %03d" % (DesiredServo, ProtocType))
    elif (DesiredData == 10):
        HomeOffset, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Homing Offset: %03d" % (DesiredServo, HomeOffset))
    elif (DesiredData == 11):
        MoveThreshold, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Threshold: %03d" % (DesiredServo, MoveThreshold))
    elif (DesiredData == 12):
        TempLimit, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Temperature Limit: %03d" % (DesiredServo, TempLimit))
    elif (DesiredData == 13):
        MaxVoltLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Max Voltage Limit: %03d" % (DesiredServo, MaxVoltLimit))
    elif (DesiredData == 14):
        MinVoltLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Min Voltage Limit: %03d" % (DesiredServo, MinVoltLimit))
    elif (DesiredData == 15):
        PWMlimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] PWM Limit: %03d" % (DesiredServo, PWMlimit))
    elif (DesiredData == 16):
        CurrLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Current Limit: %03d" % (DesiredServo, CurrLimit))
    elif (DesiredData == 17):
        AccLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Acceleration Limit: %03d" % (DesiredServo, AccLimit))
    elif (DesiredData == 18):
        VelLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity Limit: %03d" % (DesiredServo, VelLimit))
    elif (DesiredData == 19):
        MaxPosLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Max Position Limit: %03d" % (DesiredServo, MaxPosLimit))
    elif (DesiredData == 20):
        MinPosLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Min Position Limit: %03d" % (DesiredServo, MinPosLimit))
    elif (DesiredData == 21):
        ShutdownVal, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Shutdown Value: %03d" % (DesiredServo, ShutdownVal))
    elif (DesiredData == 22):
        TorqToggle, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Torque Toggle Value: %03d" % (DesiredServo, TorqToggle))
    elif (DesiredData == 23):
        LEDtoggle, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] LED Toggle Value: %03d" % (DesiredServo, LEDtoggle))
    elif (DesiredData == 24):
        StatusRetLevel, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Status Return Level: %03d" % (DesiredServo, StatusRetLevel))
    elif (DesiredData == 25):
        RegInstruction, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Registered Instruction: %03d" % (DesiredServo, RegInstruction))
    elif (DesiredData == 26):
        HardErrStat, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Hardware Error Status: %03d" % (DesiredServo, HardErrStat))
    elif (DesiredData == 27):
        VelIgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity I Gain: %03d" % (DesiredServo, VelIgain))
    elif (DesiredData == 28):
        VelPgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity P Gain: %03d" % (DesiredServo, VelPgain))
    elif (DesiredData == 29):
        PosDgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position D Gain: %03d" % (DesiredServo, PosDgain))
    elif (DesiredData == 30):
        PosIgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position I Gain: %03d" % (DesiredServo, PosIgain))
    elif (DesiredData == 31):
        PosPgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position P Gain: %03d" % (DesiredServo, PosPgain))
    elif (DesiredData == 32):
        GoalPWM, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal PWM: %03d" % (DesiredServo, GoalPWM))
    elif (DesiredData == 33):
        GoalCurr, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Current: %03d" % (DesiredServo, GoalCurr))
    elif (DesiredData == 34):
        GoalVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Velocity: %03d" % (DesiredServo, GoalVel))
    elif (DesiredData == 35):
        ProfAccel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Profile Acceleration: %03d" % (DesiredServo, ProfAccel))
    elif (DesiredData == 36):
        ProfVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Profile Velocity: %03d" % (DesiredServo, ProfVel))
    elif (DesiredData == 37):
        GoalPos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Position: %03d" % (DesiredServo, GoalPos))
    elif (DesiredData == 38):
        RealtimeTick, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Realtime Tick: %03d" % (DesiredServo, RealtimeTick))
    elif (DesiredData == 39):
        MovingVal, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Value: %03d" % (DesiredServo, MovingVal))
    elif (DesiredData == 40):
        MovingStat, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Status: %03d" % (DesiredServo, MovingStat))
    elif (DesiredData == 41):
        PresentPWM, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present PWM: %03d" % (DesiredServo, PresentPWM))
    elif (DesiredData == 42):
        PresentCurr, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Current: %03d" % (DesiredServo, PresentPWM))
    elif (DesiredData == 43):
        PresentVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Velocity: %03d" % (DesiredServo, PresentVel))
    elif (DesiredData == 44):
        PresentPos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Position: %03d" % (DesiredServo, PresentPos))
    elif (DesiredData == 45):
        VelTraj, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity Trajectory: %03d" % (DesiredServo, VelTraj))
    elif (DesiredData == 46):
        PosTraj, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position Trajectory: %03d" % (DesiredServo, PosTraj))
    elif (DesiredData == 47):
        PresInVoltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Input Voltage: %03d" % (DesiredServo, PresInVoltage))
    elif (DesiredData == 48):
        PresTemp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DesiredServo, DataAddr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Temperature: %03d" % (DesiredServo, PresTemp))
    else:
        print("Data does not have a matchable address")
    # Close port
    portHandler.closePort()

def WriteTraitData(DesiredData,DesiredValue,DesiredServo):
    # NEED TO GET RID OF THINGS THAT CAN NOT BE CHANGED!!
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

    # Change the desired trait data of the desired servo

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

    DataAddr = DataAddrConversion(DesiredData)

    if (DesiredData == 1):
        ModelNumber = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, ModelNumber)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Model Number Changed To: %03d" % (DesiredServo, ModelNumber))
    elif (DesiredData == 2):
        ModelInfo = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, ModelInfo)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Model Information Changed To: %03d" % (DesiredServo, ModelInfo))
    elif (DesiredData == 3):
        FirmwareVer = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, FirmwareVer)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Firmware Version Changed To: %03d" % (DesiredServo, FirmwareVer))
    elif (DesiredData == 4):
        IDnum = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, IDnum)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] ID Number Changed To: %03d" % (DesiredServo, IDnum))
    elif (DesiredData == 5):
        BaudRate = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, BaudRate)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Baud Rate Changed To: %03d" % (DesiredServo, BaudRate))
    elif (DesiredData == 6):
        RetDelayTime = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, RetDelayTime)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Return Delay Time Changed To: %03d" % (DesiredServo, RetDelayTime))
    elif (DesiredData == 7):
        DriveMode = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, DriveMode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Drive Mode Changed To: %03d" % (DesiredServo, DriveMode))
    elif (DesiredData == 8):
        OperateMode = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, OperateMode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Operating Mode Changed To: %03d" % (DesiredServo, OperateMode))
    elif (DesiredData == 9):
        ProtocType = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, ProtocType)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Protocol Type Changed To: %03d" % (DesiredServo, ProtocType))
    elif (DesiredData == 10):
        HomeOffset = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, HomeOffset)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Homing Offset Changed To: %03d" % (DesiredServo, HomeOffset))
    elif (DesiredData == 11):
        MoveThreshold = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, MoveThreshold)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Threshold Changed To: %03d" % (DesiredServo, MoveThreshold))
    elif (DesiredData == 12):
        TempLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, TempLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Temperature Limit Changed To: %03d" % (DesiredServo, TempLimit))
    elif (DesiredData == 13):
        MaxVoltLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, MaxVoltLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Max Voltage Limit Changed To: %03d" % (DesiredServo, MaxVoltLimit))
    elif (DesiredData == 14):
        MinVoltLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, MinVoltLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Min Voltage Limit Changed To: %03d" % (DesiredServo, MinVoltLimit))
    elif (DesiredData == 15):
        PWMlimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PWMlimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] PWM Limit Changed To: %03d" % (DesiredServo, PWMlimit))
    elif (DesiredData == 16):
        CurrLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, CurrLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Current Limit Changed To: %03d" % (DesiredServo, CurrLimit))
    elif (DesiredData == 17):
        AccLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, AccLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Acceleration Limit Changed To: %03d" % (DesiredServo, AccLimit))
    elif (DesiredData == 18):
        VelLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, VelLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity Limit Changed To: %03d" % (DesiredServo, VelLimit))
    elif (DesiredData == 19):
        MaxPosLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, MaxPosLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Max Position Limit Changed To: %03d" % (DesiredServo, MaxPosLimit))
    elif (DesiredData == 20):
        MinPosLimit = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, MinPosLimit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Min Position Limit Changed To: %03d" % (DesiredServo, MinPosLimit))
    elif (DesiredData == 21):
        ShutdownVal = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, ShutdownVal)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Shutdown Value Changed To: %03d" % (DesiredServo, ShutdownVal))
    elif (DesiredData == 22):
        TorqToggle = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, TorqToggle)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Torque Toggle Value Changed To: %03d" % (DesiredServo, TorqToggle))
    elif (DesiredData == 23):
        LEDtoggle = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, LEDtoggle)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] LED Toggle Value Changed To: %03d" % (DesiredServo, LEDtoggle))
    elif (DesiredData == 24):
        StatusRetLevel = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, StatusRetLevel)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Status Return Level Changed To: %03d" % (DesiredServo, StatusRetLevel))
    elif (DesiredData == 25):
        RegInstruction = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, RegInstruction)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Registered Instruction Changed To: %03d" % (DesiredServo, RegInstruction))
    elif (DesiredData == 26):
        HardErrStat = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, HardErrStat)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Hardware Error Status Changed To: %03d" % (DesiredServo, HardErrStat))
    elif (DesiredData == 27):
        VelIgain = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, VelIgain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity I Gain Changed To: %03d" % (DesiredServo, VelIgain))
    elif (DesiredData == 28):
        VelPgain = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, VelPgain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity P Gain Changed To: %03d" % (DesiredServo, VelPgain))
    elif (DesiredData == 29):
        PosDgain = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PosDgain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position D Gain Changed To: %03d" % (DesiredServo, PosDgain))
    elif (DesiredData == 30):
        PosIgain = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PosIgain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position I Gain Changed To: %03d" % (DesiredServo, PosIgain))
    elif (DesiredData == 31):
        PosPgain = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PosPgain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position P Gain Changed To: %03d" % (DesiredServo, PosPgain))
    elif (DesiredData == 32):
        GoalPWM = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, GoalPWM)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal PWM Changed To: %03d" % (DesiredServo, GoalPWM))
    elif (DesiredData == 33):
        GoalCurr = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, GoalCurr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Current Changed To: %03d" % (DesiredServo, GoalCurr))
    elif (DesiredData == 34):
        GoalVel = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, GoalVel)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Velocity Changed To: %03d" % (DesiredServo, GoalVel))
    elif (DesiredData == 35):
        ProfAccel = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, ProfAccel)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Profile Acceleration Changed To: %03d" % (DesiredServo, ProfAccel))
    elif (DesiredData == 36):
        ProfVel = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, ProfVel)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Profile Velocity Changed To: %03d" % (DesiredServo, ProfVel))
    elif (DesiredData == 37):
        GoalPos = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, GoalPos)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Goal Position Changed To: %03d" % (DesiredServo, GoalPos))
    elif (DesiredData == 38):
        RealtimeTick = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, RealtimeTick)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Realtime Tick Changed To: %03d" % (DesiredServo, RealtimeTick))
    elif (DesiredData == 39):
        MovingVal = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, MovingVal)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Value Changed To: %03d" % (DesiredServo, MovingVal))
    elif (DesiredData == 40):
        MovingStat = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, MovingStat)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Moving Status Changed To: %03d" % (DesiredServo, MovingStat))
    elif (DesiredData == 41):
        PresentPWM = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PresentPWM)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present PWM Changed To: %03d" % (DesiredServo, PresentPWM))
    elif (DesiredData == 42):
        PresentCurr = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PresentCurr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Current Changed To: %03d" % (DesiredServo, PresentPWM))
    elif (DesiredData == 43):
        PresentVel = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, PresentVel)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Velocity Changed To: %03d" % (DesiredServo, PresentVel))
    elif (DesiredData == 44):
        PresentPos = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, PresentPos)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Position Changed To: %03d" % (DesiredServo, PresentPos))
    elif (DesiredData == 45):
        VelTraj = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, VelTraj)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Velocity Trajectory Changed To: %03d" % (DesiredServo, VelTraj))
    elif (DesiredData == 46):
        PosTraj = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DesiredServo, DataAddr, PosTraj)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Position Trajectory Changed To: %03d" % (DesiredServo, PosTraj))
    elif (DesiredData == 47):
        PresInVoltage = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DesiredServo, DataAddr, PresInVoltage)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Input Voltage Changed To: %03d" % (DesiredServo, PresInVoltage))
    elif (DesiredData == 48):
        PresTemp = DesiredValue
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DesiredServo, DataAddr, PresTemp)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("[ID:%03d] Present Temperature Changed To: %03d" % (DesiredServo, PresTemp))
    else:
        print("Data does not have a matchable address")

    # Close port
    portHandler.closePort()

def RebootServos(DesiredServo):
    #The Reboot function can be used when the Dynamixel stops moving since the Dynamixel error occurred by, for example, overload.

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

    # Trigger
    print("Press any key to reboot")
    getch()

    print("See the Dynamixel LED flickering")
    # Try reboot
    # Dynamixel LED will flicker while it reboots
    dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, DesiredServo)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("[ID:%03d] reboot Succeeded\n" % DesiredServo)


    # Close port
    portHandler.closePort()

def ResetServos(DesiredServo):
    #resets settings of Dynamixel to default values. The Factoryreset function has three operation modes:
    #0xFF : reset all values (ID to 1, baudrate to 57600)
    #0x01 : reset all values except ID (baudrate to 57600)
    #0x02 : reset all values except ID and baudrate.

    import os, sys

    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        def getch():
            return sys.stdin.read(1)
    
    os.sys.path.append('../dynamixel_functions_py')             # Path setting
    
    from time import sleep
    import dynamixel_functions as dynamixel                     # Uses DYNAMIXEL SDK library

    # Initialize PortHandler Structs
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    port_num = dynamixel.portHandler(DEVICENAME)

    # Initialize PacketHandler Structs
    dynamixel.packetHandler()

    dxl_comm_result = COMM_TX_FAIL                              # Communication result

    dxl_error = 0                                               # Dynamixel error
    dxl_baudnum_read = 0                                        # Read baudnum

    # Open port
    if dynamixel.openPort(port_num):
        print("Succeeded to open the port!")
    else:
        print("Failed to open the port!")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeeded to change the baudrate!")
    else:
        print("Failed to change the baudrate!")
        print("Press any key to terminate...")
        getch()
        quit()


    # Read present baudrate of the controller
    print("Now the controller baudrate is : %d" % (dynamixel.getBaudRate(port_num)))

    # Try factoryreset
    print("[ID:%03d] Try factoryreset : " % (DesiredServo))
    dynamixel.factoryReset(port_num, PROTOCOL_VERSION, DesiredServo, OPERATION_MODE)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        print("Aborted")
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        quit()
    elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))


    # Wait for reset
    print("Wait for reset...")
    sleep(2)

    print("[ID:%03d] factoryReset Success!" % (DesiredServo))

    # Set controller baudrate to dxl default baudrate
    if dynamixel.setBaudRate(port_num, FACTORYRST_DEFAULTBAUDRATE):
        print("Succeed to change the controller baudrate to : %d" % (FACTORYRST_DEFAULTBAUDRATE))
    else:
        print("Failed to change the controller baudrate")
        getch()
        quit()

    # Read Dynamixel baudnum
    dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DesiredServo, ADDR_PRO_BAUDRATE)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
    elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
    else:
      print("[ID:%03d] Dynamixel baudnum is now : %d" % (DesiredServo, dxl_baudnum_read))

    # Write new baudnum
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DesiredServo, ADDR_PRO_BAUDRATE, NEW_BAUDNUM)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
    elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
    else:
      print("[ID:%03d] Set Dynamixel baudnum to : %d" % (DesiredServo, NEW_BAUDNUM))

    # Set port baudrate to BAUDRATE
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeed to change the controller baudrate to : %d" % (BAUDRATE))
    else:
        print("Failed to change the controller baudrate")
        getch()
        quit()

    sleep(0.2)

    # Read Dynamixel baudnum
    dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DesiredServo, ADDR_PRO_BAUDRATE)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
    elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
    else:
      print("[ID:%03d] Dynamixel baudnum is now : %d" % (DesiredServo, dxl_baudnum_read))


    # Close port
    dynamixel.closePort(port_num)

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