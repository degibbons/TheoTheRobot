import os
from ControlTable import *
from dynamixel_sdk import *
import numpy as np
import pandas as pd
import copy as cp
import time
from curtsies import Input
from threading import Thread
import RPi.GPIO as GPIO
import math
import sys
#import dynamixel_functions as dynamixel  
import csv

global stopVal
stopVal = 0
global t1
global t2

################################ CLASS SECTION ####################################################
class Servo:
    def __init__(self,IDnum,Positions):
        self.ID = IDnum
        
        #self.portHandler = PortHandler(DEVICENAME_1)

        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.Positions = Positions
        self.Speeds = []
        self.HomeSpeed = HOME_SPEED
        self.IsHome = None
        self.Activated = 1 # Change this to zero if you don't want it to move
        self.FirstMovePosition = 0
        #self.OffsetPercent = 0
        #self.Phase = 0 # 2 for Stance, 1 for transition to stance, -2 for swing, -1 for transition to swing, 0 for other
        self.PresentPosition = 0
        self.GivenPosition = 0
        self.GivenSpeed = 0
        self.MovementTime = 0 # Length of time for the last movement given
        #self.PhaseIndex = 0 
        self.StrideIndex = 1
        #self.PhaseTime = 0
        #self.StrideTime = 0
        #self.TotalTime = 0
        #self.IndexShifts = RotatePositionArray(list(range(0,22)),self.OffsetPercent/10,len(list(range(0,22))))
        self.DataArray = []

    def InitialSetup(self,portHandler): 

        #Set drive mode to velocity based
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_DRIVE_MODE, DRIVE_MODE_VEL_BASED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Drive mode set to: %03d" %(self.ID, DRIVE_MODE_VEL_BASED))

        time.sleep(PreferedDelay)

        #Set operating mode to joint/position control mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_OPERATING_MODE, OPERATING_JOINT_POSITION_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Operating mode set to: %03d" %(self.ID, OPERATING_JOINT_POSITION_MODE))

        time.sleep(PreferedDelay)

        #Set acceleration limit 
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_ACCELERATION_LIMIT, ACCELERATION_LIMIT_M)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Acceleration limit set to: %03d" %(self.ID, ACCELERATION_LIMIT_M))

        time.sleep(PreferedDelay)

        #Set max position limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Max position limit set to: %03d" %(self.ID, MAX_POSITION_LIMIT))

        time.sleep(PreferedDelay)

        #Set min position limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Min position limit set to: %03d" %(self.ID, MIN_POSITION_LIMIT))

        time.sleep(PreferedDelay)

        #Set moving threshold
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD_ACCURACY_H)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Moving accuracy set to high: %03d" %(self.ID, MOVING_THRESHOLD_ACCURACY_H))

        time.sleep(PreferedDelay)

    def ToggleTorque(self,OnOrOff,portHandler,packetHandler):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_PRO_TORQUE_ENABLE, OnOrOff)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            if (OnOrOff == 1):
                print("Dynamixel#%d torque on" % self.ID)
            elif (OnOrOff == 0):
                print("Dynamixel#%d torque off" % self.ID)

    def SetServoVelocity(self,InVelocity,portHandler):
        print(InVelocity)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_PROFILE_VELOCITY,int(InVelocity))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Velocity limit set to: %03d" %(self.ID, InVelocity))

    def MoveServo(self,InPosition,portHandler):
         # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_PRO_GOAL_POSITION, InPosition)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Goal Position set to: %03d" %(self.ID, InPosition))

    def MoveHome(self):
        self.SetServoVelocity(self.HomeSpeed)
        self.MoveServo(self.Positions[0])


    def ContinuousMove(self,portHandler,packetHandler,DataRecord,CurrentDoc):
        global stopVal
        index = 1
        while 1:
            if (index > 21):
                index = 0
            elif (index == 1):
                self.StrideIndex += 1

            self.SetServoVelocity(self.Speeds[index])
            self.MoveServo(self.Positions[index])
            if (stopVal == 1):
                print("\nFinishing Movement.\n")
                return
            while 1:
                dxl_mov, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(portHandler, self.ID, ADDR_MOVING)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                if dxl_mov == 1:
                    pass
                else:
                    break
            index += 1

    def RebootServo(self,portHandler):
        import os

        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            
            import sys, tty, termios # pylint: disable=import-error
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            return

        # Trigger
        print("Press any key to reboot")
        getch()

        print("See the Dynamixel LED flickering")
        # Try reboot
        # Dynamixel LED will flicker while it reboots
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, self.ID)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] reboot Succeeded\n" % self.ID)


        # Close port
        #portHandler.closePort()

    def ResetServo(self,portHandler):
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
            import tty, termios # pylint: disable=import-error
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(sys.stdin.fileno())
            def getch():
                return sys.stdin.read(1)

        os.sys.path.append('../dynamixel_functions_py')             # Path setting

                           # Uses DYNAMIXEL SDK library

        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        port_num = dynamixel.portHandler(DEVICENAME_1)

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
            return

        # Set port baudrate
        if dynamixel.setBaudRate(port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            print("Press any key to terminate...")
            getch()
            return

        # Read present baudrate of the controller
        print("Now the controller baudrate is : %d" % (dynamixel.getBaudRate(port_num)))

        # Try factoryreset
        print("[ID:%03d] Try factoryreset : " % (self.ID))
        dynamixel.factoryReset(port_num, PROTOCOL_VERSION, self.ID, OPERATION_MODE)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            print("Aborted")
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
            return
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))


        # Wait for reset
        print("Wait for reset...")
        time.sleep(2)

        print("[ID:%03d] factoryReset Success!" % (self.ID))

        # Set controller baudrate to dxl default baudrate
        if dynamixel.setBaudRate(port_num, FACTORYRST_DEFAULTBAUDRATE):
            print("Succeed to change the controller baudrate to : %d" % (FACTORYRST_DEFAULTBAUDRATE))
        else:
            print("Failed to change the controller baudrate")
            getch()
            return

        # Read Dynamixel baudnum
        dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, self.ID, ADDR_PRO_BAUDRATE)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
        else:
          print("[ID:%03d] Dynamixel baudnum is now : %d" % (self.ID, dxl_baudnum_read))

        # Write new baudnum
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, self.ID, ADDR_PRO_BAUDRATE, NEW_BAUDNUM)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
        else:
          print("[ID:%03d] Set Dynamixel baudnum to : %d" % (self.ID, NEW_BAUDNUM))

        # Set port baudrate to BAUDRATE
        if dynamixel.setBaudRate(port_num, BAUDRATE):
            print("Succeed to change the controller baudrate to : %d" % (BAUDRATE))
        else:
            print("Failed to change the controller baudrate")
            getch()
            return

        time.sleep(0.2)

        # Read Dynamixel baudnum
        dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, self.ID, ADDR_PRO_BAUDRATE)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
        else:
          print("[ID:%03d] Dynamixel baudnum is now : %d" % (self.ID, dxl_baudnum_read))


        # Close port
        dynamixel.closePort(port_num)

    def CleanUp(self):
        pass

    def __del__(self):
        pass

class Limb:
    def __init__(self,limbnum,servolist):
        self.LimbNumber = limbnum

        self.ServoList = servolist

        getNum = lambda a : a.ID
        self.IDList = list(map(getNum,self.ServoList))

        listLen = len(self.IDList)
        rangeList = list(range(0,listLen))
        self.ServoDict = {}

        for i in rangeList:
            self.ServoDict[i+1] = self.ServoList[i] 

        self.IsHome = None # Check all Servo Members to set this value
        self.FirstMovePosition = None # Check all Servo Members to set this value
        self.TotalTime = 0
        self.HomeSpeed = HOME_SPEED
        self.GoalVelocity = []
        self.GoalPosition = []
        self.PresentPositions = []


class Leg(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

        self.Phase = 0 # Check all Servo Members to set this value
        self.PhaseTime = 0 
        self.StrideTime = 0
        self.StrideCount = 1
        # offsets = []
        # for k in self.ServoList:
        #     offsets.append(k.OffsetPercent)
        # if len(set(offsets))==1:
        #     self.Offset = offsets[0]
        # else:
        #     print("The Offsets for the Servos that make up this Leg are not consistent. Please Fix.")
        # self.IndexShifts = RotatePositionArray(list(range(0,22)),self.Offset/10,len(list(range(0,22))))
        # self.DataArray1 = []
        # self.DataArray2 = []
        # self.DataArray3 = []
        # self.DataArray4 = []

        # self.OnceOrCont = 0

    def MoveLimb(self,IndexIn,portHandler,packetHandler,isHomeMovement):

        # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity = []
        self.GoalPosition = []
        P_IndexIn = IndexIn
        if (IndexIn == 0):
            S_IndexIn = 21
        else:
            S_IndexIn = IndexIn-1
              
        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(int(b.Speeds[S_IndexIn])))
            self.GoalPosition.append(FormatSendData(b.Positions[P_IndexIn]))

        index = 0
        for _ , d in self.ServoDict.items():
            FormattedVel = self.GoalVelocity[index]
            if isHomeMovement == True:
                FormattedVel = FormatSendData(self.HomeSpeed)
            dxl_addparam_result = groupSyncWriteVEL.addParam(d.ID,FormattedVel)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return

            FormattedPos = self.GoalPosition[index]
            dxl_addparam_result = groupSyncWritePOS.addParam(d.ID,FormattedPos)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return
                
            index += 1

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

    def MoveHome(self,portHandler,packetHandler):
        self.MoveLimb(0,portHandler,packetHandler,True)
        self.IsHome = True
        for i in self.IDList:
            print("Moving Servo#{sn} home. ".format(sn=i))
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler):
        global stopVal
        # Initialize GroupSyncRead instace for Present Position
        groupSyncReadMOV = GroupSyncRead(portHandler, packetHandler, ADDR_MOVING, LEN_MOVING)
        index = 1
        with open('SpeedPosMatching.csv', 'a', newline='') as csvfile:
            DocWriter = csv.writer(csvfile, delimiter=',',quoting=csv.QUOTE_MINIMAL)
            for x in range(0,51):
                self.MoveLimb(index,portHandler,packetHandler,False)
                if (index == 0):
                    S_IndexIn = 21
                else:
                    S_IndexIn = index-1
                P_IndexIn = index
                PositionList = []
                SpeedList = []
                for b in self.ServoList:
                    Pos_num = b.Positions[P_IndexIn]
                    Sp_num = int(b.Speeds[S_IndexIn])
                    PositionList.append(Pos_num)
                    SpeedList.append(Sp_num)
                CombinedList = list(zip(PositionList,SpeedList))
                print("-------------------------")
                print("Index Movement is: {ind}".format(ind = index))
                print("-------------------------")
                index += 1
                if (index > 21): 
                    index = 0
                elif (index == 1):
                    print("==================================================")
                    print("Stride Number: {sn}".format(sn=self.StrideCount))
                    print("==================================================")
                    self.StrideCount += 1
                isStopped = [0] * len(self.ServoList)
                for i in self.IDList:
                    groupSyncReadMOV.addParam(i)
                while 1:
                    # Syncread Moving Value
                    dxl_comm_result = groupSyncReadMOV.txRxPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    if (stopVal == 1):
                        print("\nFinishing All Movement.\n")
                        return

                    for count,i in enumerate(self.IDList):
                        # Get Dynamixel#1 present Moving value
                        dxl_mov = groupSyncReadMOV.getData(i, ADDR_MOVING, LEN_MOVING)               
                        if (dxl_mov == 0) and (isStopped[count] == 0):
                            isStopped[count] = 1

                    if 0 in isStopped:
                        pass
                    else:
                        DocWriter.writerow(CombinedList)
                        break
                groupSyncReadMOV.clearParam()
        print("\nDone Writing! Please Hit Esc.\n\n")

    # def UpdateOtherValues(self,IndexIn):
    #     self.GoalVelocity = []
    #     self.GoalPosition = []

    #     for b in self.ServoList:
    #         self.GoalVelocity.append(b.Speeds[IndexIn])
    #         self.GoalPosition.append(b.Positions[IndexIn])

    def __del__(self):
            pass        


class Neck(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)
        
    def MoveLimb(self,IndexIn,portHandler,packetHandler,isHomeMovement):
        # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity = []
        self.GoalPosition = []
              
        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(int(STRAIGHT_SPEED)))
            self.GoalPosition.append(FormatSendData(int(STRAIGHT_SPINE)))

        index = 0
        for _ , d in self.ServoDict.items():
            FormattedVel = self.GoalVelocity[index]
            if isHomeMovement == True:
                FormattedVel = FormatSendData(self.HomeSpeed)
            dxl_addparam_result = groupSyncWriteVEL.addParam(d.ID,FormattedVel)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return

            FormattedPos = self.GoalPosition[index]
            dxl_addparam_result = groupSyncWritePOS.addParam(d.ID,FormattedPos)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return
                
            index += 1

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

    def MoveHome(self,portHandler,packetHandler):
        self.MoveLimb(0,portHandler,packetHandler,True)
        self.IsHome = True
        for i in self.IDList:
            print("Moving Servo#{sn} home. ".format(sn=i))
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler):
        pass

    def __del__(self):
        pass  

class Spine(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

    def MoveLimb(self,IndexIn,portHandler,packetHandler,isHomeMovement):
        # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity = []
        self.GoalPosition = []
              
        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(int(STRAIGHT_SPEED)))
            self.GoalPosition.append(FormatSendData(int(STRAIGHT_SPINE)))

        index = 0
        for _ , d in self.ServoDict.items():
            FormattedVel = self.GoalVelocity[index]
            if isHomeMovement == True:
                FormattedVel = FormatSendData(self.HomeSpeed)
            dxl_addparam_result = groupSyncWriteVEL.addParam(d.ID,FormattedVel)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return

            FormattedPos = self.GoalPosition[index]
            dxl_addparam_result = groupSyncWritePOS.addParam(d.ID,FormattedPos)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return
                
            index += 1

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

    def MoveHome(self,portHandler,packetHandler):
        self.MoveLimb(0,portHandler,packetHandler,True)
        self.IsHome = True
        for i in self.IDList:
            print("Moving Servo#{sn} home. ".format(sn=i))
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler):
        pass
    def __del__(self):
        pass  

class Tail(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

    def MoveLimb(self,IndexIn,portHandler,packetHandler,isHomeMovement):
        # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity = []
        self.GoalPosition = []
              
        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(int(STRAIGHT_SPEED)))
            self.GoalPosition.append(FormatSendData(int(STRAIGHT_SPINE)))

        index = 0
        for _ , d in self.ServoDict.items():
            FormattedVel = self.GoalVelocity[index]
            if isHomeMovement == True:
                FormattedVel = FormatSendData(self.HomeSpeed)
            dxl_addparam_result = groupSyncWriteVEL.addParam(d.ID,FormattedVel)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return

            FormattedPos = self.GoalPosition[index]
            dxl_addparam_result = groupSyncWritePOS.addParam(d.ID,FormattedPos)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d.ID)
                return
                
            index += 1

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

    def MoveHome(self,portHandler,packetHandler):
        self.MoveLimb(0,portHandler,packetHandler,True)
        self.IsHome = True
        for i in self.IDList:
            print("Moving Servo#{sn} home. ".format(sn=i))
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler):
        pass
    def __del__(self):
        pass  

class Body:
    def __init__(self,limbs): #limbs should be a list
        self.limbs = limbs
        self.GoalVelocity_FR = []
        self.GoalPosition_FR = []
        self.GoalVelocity_FL = []
        self.GoalPosition_FL = []
        self.GoalVelocity_BR = []
        self.GoalPosition_BR = []
        self.GoalVelocity_BL = []
        self.GoalPosition_BL = []
        self.isHome = False
        self.HomeSpeed = HOME_SPEED
        self.StrideCount = 1

    def MoveLegs(self,IndexIn,portHandler1,portHandler2,packetHandler,isHomeMovement):
         # Initialize GroupSyncWrite instance
        groupSyncWritePOS_Front = GroupSyncWrite(portHandler1, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL_Front = GroupSyncWrite(portHandler1, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)
        # Initialize GroupSyncWrite instance
        groupSyncWritePOS_Back = GroupSyncWrite(portHandler2, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL_Back = GroupSyncWrite(portHandler2, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity_FR = []
        self.GoalPosition_FR = []
        self.GoalVelocity_FL = []
        self.GoalPosition_FL = []
        self.GoalVelocity_BR = []
        self.GoalPosition_BR = []
        self.GoalVelocity_BL = []
        self.GoalPosition_BL = []
        P_IndexIn = IndexIn
        
        if (IndexIn == 0):
            S_IndexIn = 21
        else:
            S_IndexIn = IndexIn-1

        for limbX in self.limbs:
            if (limbX.LimbNumber == 1):
                for servoX in limbX.ServoList:
                    self.GoalVelocity_FR.append(FormatSendData(int(servoX.Speeds[S_IndexIn])))
                    self.GoalPosition_FR.append(FormatSendData(servoX.Positions[P_IndexIn]))
            elif(limbX.LimbNumber == 2):
                for servoX in limbX.ServoList:
                    self.GoalVelocity_FL.append(FormatSendData(int(servoX.Speeds[S_IndexIn])))
                    self.GoalPosition_FL.append(FormatSendData(servoX.Positions[P_IndexIn]))
            elif (limbX.LimbNumber == 3):
                for servoX in limbX.ServoList:
                    self.GoalVelocity_BL.append(FormatSendData(int(servoX.Speeds[S_IndexIn])))
                    self.GoalPosition_BL.append(FormatSendData(servoX.Positions[P_IndexIn]))
            elif (limbX.LimbNumber == 4):
                for servoX in limbX.ServoList:
                    self.GoalVelocity_BL.append(FormatSendData(int(servoX.Speeds[S_IndexIn])))
                    self.GoalPosition_BL.append(FormatSendData(servoX.Positions[P_IndexIn]))
            else:
                pass

        index = 0
        for limbX in self.limbs:
            if (limbX.LimbNumber == 1): 
                for servoX in limbX.ServoList:
                    FormattedVel = self.GoalVelocity_FR[index]
                    if isHomeMovement == True:
                        FormattedVel = FormatSendData(self.HomeSpeed)
                    dxl_addparam_result = groupSyncWriteVEL_Front.addParam(servoX.ID,FormattedVel)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
                    FormattedPos = self.GoalPosition_FR[index]
                    dxl_addparam_result = groupSyncWritePOS_Front.addParam(servoX.ID,FormattedPos)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
            elif (limbX.LimbNumber == 2):
                for servoX in limbX.ServoList:
                    FormattedVel = self.GoalVelocity_FL[index]
                    if isHomeMovement == True:
                        FormattedVel = FormatSendData(self.HomeSpeed)
                    dxl_addparam_result = groupSyncWriteVEL_Front.addParam(servoX.ID,FormattedVel)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
                    FormattedPos = self.GoalPosition_FL[index]
                    dxl_addparam_result = groupSyncWritePOS_Front.addParam(servoX.ID,FormattedPos)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
            elif (limbX.LimbNumber == 3):
                for servoX in limbX.ServoList:
                    FormattedVel = self.GoalVelocity_BR[index]
                    if isHomeMovement == True:
                        FormattedVel = FormatSendData(self.HomeSpeed)
                    dxl_addparam_result = groupSyncWriteVEL_Back.addParam(servoX.ID,FormattedVel)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
                    FormattedPos = self.GoalPosition_BR[index]
                    dxl_addparam_result = groupSyncWritePOS_Back.addParam(servoX.ID,FormattedPos)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
            elif (limbX.LimbNumber == 4):
                for servoX in limbX.ServoList:
                    FormattedVel = self.GoalVelocity_BL[index]
                    if isHomeMovement == True:
                        FormattedVel = FormatSendData(self.HomeSpeed)
                    dxl_addparam_result = groupSyncWriteVEL_Back.addParam(servoX.ID,FormattedVel)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
                    FormattedPos = self.GoalPosition_BL[index]
                    dxl_addparam_result = groupSyncWritePOS_Back.addParam(servoX.ID,FormattedPos)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupSyncWrite addparam failed" % servoX.ID)
                        return
            index += 1

            # Syncwrite goal velocity
            dxl_comm_result = groupSyncWriteVEL_Front.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Syncwrite goal velocity
            dxl_comm_result = groupSyncWriteVEL_Back.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Clear syncwrite parameter storage
            groupSyncWriteVEL_Front.clearParam()

            groupSyncWriteVEL_Back.clearParam()

            # Syncwrite goal position
            dxl_comm_result = groupSyncWritePOS_Front.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Syncwrite goal position
            dxl_comm_result = groupSyncWritePOS_Back.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))            

            # Clear syncwrite parameter storage
            groupSyncWritePOS_Front.clearParam()

            groupSyncWritePOS_Back.clearParam()


    def MoveLegsHome(self,portHandler1,portHandler2,packetHandler):
        self.MoveLegs(0,portHandler1,portHandler2,packetHandler,True)
        self.IsHome = True
        for limbX in LegLimbs:
            print("Moving Limb #{sn} ({nm}) Home. ".format(sn=limbX,nm=LimbNames[i]))

    def MoveSpineHome(self,portHandler3,packetHandler):
        self.limbs[4].MoveHome(portHandler3,packetHandler)
        self.limbs[5].MoveHome(portHandler3,packetHandler)
        self.limbs[6].MoveHome(portHandler3,packetHandler)
        self.IsHome = True
        for limbX in MainBodyLimbs:
            print("Moving Limb #{sn} ({nm}) Home. ".format(sn=limbX,nm=MainBodyLimbs[i]))

    def ContinuousLegsMove(self,portHandler1,portHandler2,packetHandler):
        global stopVal
        # Initialize GroupSyncRead instace for Present Position
        groupSyncReadMOV_Fr = GroupSyncRead(portHandler1, packetHandler, ADDR_MOVING, LEN_MOVING)
        groupSyncReadMOV_Ba = GroupSyncRead(portHandler2, packetHandler, ADDR_MOVING, LEN_MOVING)
        index = [1] * len(self.limbs)
        innerIndex = 0
        with open('SpeedPosMatching.csv', 'a', newline='') as csvfile:
            DocWriter = csv.writer(csvfile, delimiter=',',quoting=csv.QUOTE_MINIMAL)
            for x in range(0,51):
                self.MoveLegs(index,portHandler1,portHandler2,packetHandler,False)
                if (index[innerIndex] == 0):
                    S_IndexIn = 21
                else:
                    S_IndexIn = index[innerIndex]-1
                P_IndexIn = index
                PositionList = []
                SpeedList = []
                for b in limbX.ServoList:
                    Pos_num = b.Positions[P_IndexIn]
                    Sp_num = int(b.Speeds[S_IndexIn])
                    PositionList.append(Pos_num)
                    SpeedList.append(Sp_num)
                CombinedList = list(zip(PositionList,SpeedList))
                for limbX in self.limbs:
                    print("-------------------------")
                    print("Index Movement for limb {lnum} is: {ind}".format(ind = index, lnum = limbX.LimbNumber))
                    print("-------------------------")
                index[innerIndex] += 1
                if (index[innerIndex] > 21): 
                    index[innerIndex] = 0
                elif (index[innerIndex] == 1):
                    print("==================================================")
                    print("Stride Number: {sn}".format(sn=self.StrideCount))
                    print("==================================================")
                    limbX.StrideCount += 1
                isStopped = [0] * 16
                for limbX in self.limbs:
                    if (limbX == 1 or limbX == 2):
                        for i in limbX.IDList:
                            groupSyncReadMOV_Fr.addParam(i)
                    elif (limbX == 3 or limbX == 4):
                        for i in limbX.IDList:
                            groupSyncReadMOV_Ba.addParam(i)
                while 1:
                    # Syncread Moving Value
                    dxl_comm_result_Fr = groupSyncReadMOV_Fr.txRxPacket()
                    dxl_comm_result_Ba = groupSyncReadMOV_Ba.txRxPacket()
                    if dxl_comm_result_Fr != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    if dxl_comm_result_Ba != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    
                    if (stopVal == 1):
                        print("\nFinishing All Movement.\n")
                        return

                    for count,i in enumerate(list(range(1,17))):
                        if (i >= 1 and i <= 8):
                            dxl_mov = groupSyncReadMOV_Fr.getData(i, ADDR_MOVING, LEN_MOVING)
                        elif (i > 8 and i <= 16):
                            dxl_mov = groupSyncReadMOV_Ba.getData(i, ADDR_MOVING, LEN_MOVING)
                        if (dxl_mov == 0) and (isStopped[count] == 0):
                            isStopped[count] = 1

                    if 0 in isStopped:
                        pass
                    else:
                        DocWriter.writerow(CombinedList)
                        break
                groupSyncReadMOV_Fr.clearParam()
                groupSyncReadMOV_Ba.clearParam()
        print("\nDone Writing! Please Hit Esc.\n\n")

    def __del__(self):
            pass

################################ FUNCTIONS SECTION ####################################################

def InitialSetup():
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler_1 = PortHandler(DEVICENAME_1)
    portHandler_2 = PortHandler(DEVICENAME_2)
    portHandler_3 = PortHandler(DEVICENAME_3)
    portHandler_4 = PortHandler(DEVICENAME_4)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler_1.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Open port
    if portHandler_2.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Open port
    if portHandler_3.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Open port
    if portHandler_4.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Set port baudrate
    if portHandler_1.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Set port baudrate
    if portHandler_2.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    # Set port baudrate
    if portHandler_3.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return
    
    # Set port baudrate
    if portHandler_4.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch() # pylint: disable=undefined-variable
        return

    return portHandler_1, portHandler_2, portHandler_3, portHandler_4, packetHandler

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
    global stopVal
    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            if (str(e) == '\x1b'):
                print("Stopping Servo Movement\n")
                stopVal = 1
                break

def DetermineSpeeds(tspan,PositionsMatrix):
    #Import position points as a data frame (df)
    #old_df = pd.read_csv(positionsFile)
    #old_df = old_df.values[:][:]
    #Make a copy of the dataframe with the same dimensions for the speeds
    speeds = cp.copy(PositionsMatrix)
    h_stance = 4.892994
    h_swing = 3.347006
    f_stance = 5.211718
    f_swing = 3.028282
    h_st_per = h_stance / 8.24
    h_sw_per = h_swing / 8.24
    f_st_per = f_stance / 8.24
    f_sw_per = f_swing / 8.24
    #Starting % index for each limb
    L_hl = 0
    L_fl = 9 # 90% Approximately
    R_hl = 8 # 80% Approximately
    R_fl = 1 # 10% Approximately
    #Points per Phase
    pointsPerPhase = 11
    speeds = np.array(speeds)
    b = speeds.shape
    cLength = int(b[1]/2)
    cWidth = int(b[0])
    MoveIndex = np.linspace(1,int(b[1]),int(b[1]))
    servos = list(range(1,cWidth+1))
    MoveIndex = MoveIndex.astype(int).tolist()
    newSpeeds = []
    #t1 = np.zeros(b[1])
    for i in np.linspace(1,cWidth,cWidth):
        newSpeeds.append(np.zeros(b[1]))
    for i in servos:
        for j in MoveIndex: 
            if (i == 1 or i == 2 or i == 3 or i == 4 ):
                if (j==b[1]):
                    rotations = abs(speeds[i-1][0]-speeds[i-1][j-1])/4096
                else:
                    rotations = abs(speeds[i-1][j]-speeds[i-1][j-1])/4096
                if ((j-1) >= (pointsPerPhase - R_fl) and (j-1)<=(pointsPerPhase - R_fl + 10)):
                    movementTime = (tspan*f_sw_per/cLength)/60
                else:
                    movementTime = (tspan*f_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                newSpeeds[i-1][j-1] = round(movementSpeed)
            elif (i == 5 or i == 6 or i == 7 or i == 8): 
                if (j==b[1]):
                    rotations = abs(speeds[i-1][0]-speeds[i-1][j-1])/4096
                else:
                    rotations = abs(speeds[i-1][j]-speeds[i-1][j-1])/4096
                if ((j-1) >= (pointsPerPhase - L_fl) and (j-1)<=(pointsPerPhase - L_fl + 10)):
                    movementTime = (tspan*f_sw_per/cLength)/60
                else:
                    movementTime = (tspan*f_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                newSpeeds[i-1][j-1] = round(movementSpeed)
            elif (i == 9 or i == 10 or i == 11 or i == 12 ):
                if (j==b[1]):
                    rotations = abs(speeds[i-1][0]-speeds[i-1][j-1])/4096
                else:
                    rotations = abs(speeds[i-1][j]-speeds[i-1][j-1])/4096
                if ((j-1) >= (pointsPerPhase - R_hl) and (j-1)<=(pointsPerPhase - R_hl + 10)):
                    movementTime = (tspan*h_sw_per/cLength)/60
                else:
                    movementTime = (tspan*h_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                newSpeeds[i-1][j-1] = round(movementSpeed)
            elif (i == 13 or i == 14 or i == 15 or i == 16 ):
                if (j==b[1]):
                    rotations = abs(speeds[i-1][0]-speeds[i-1][j-1])/4096
                else:
                    rotations = abs(speeds[i-1][j]-speeds[i-1][j-1])/4096
                if ((j-1) >= (pointsPerPhase - L_hl) and (j-1)<=(pointsPerPhase - L_hl + 10)):
                    movementTime = (tspan*h_sw_per/cLength)/60
                else:
                    movementTime = (tspan*h_st_per/cLength)/60
                movementSpeed = (rotations / movementTime) / 0.114
                newSpeeds[i-1][j-1] = round(movementSpeed)
    for i in servos:
        newSpeeds[i-1] = newSpeeds[i-1].astype(int)
    for j in servos:
        newSpeeds[j-1][newSpeeds[j-1] > 1023] = 1023
        newSpeeds[j-1][newSpeeds[j-1] == 0] = 1

    return newSpeeds


def ReadServoAngles(positionsFile):

    old_df = pd.read_csv(positionsFile)
    old_df = old_df.values[:][:]

    FL_ST_R_1 = old_df[:,0]
    FL_ST_R_2 = old_df[:,1]
    FL_ST_R_3 = old_df[:,2]
    FL_ST_R_4 = old_df[:,3]

    FL_SW_R_1 = old_df[:,4]
    FL_SW_R_2 = old_df[:,5]
    FL_SW_R_3 = old_df[:,6]
    FL_SW_R_4 = old_df[:,7]

    FL_TOT_R_1 = np.concatenate((FL_ST_R_1, FL_SW_R_1),axis=0)
    FL_TOT_R_2 = np.concatenate((FL_ST_R_2, FL_SW_R_2),axis=0)
    FL_TOT_R_3 = np.concatenate((FL_ST_R_3, FL_SW_R_3),axis=0)
    FL_TOT_R_4 = np.concatenate((FL_ST_R_4, FL_SW_R_4),axis=0)

    FL_ST_L_1 = old_df[:,0]
    FL_ST_L_2 = old_df[:,1]
    FL_ST_L_3 = old_df[:,2]
    FL_ST_L_4 = old_df[:,3]

    FL_SW_L_1 = old_df[:,4]
    FL_SW_L_2 = old_df[:,5]
    FL_SW_L_3 = old_df[:,6]
    FL_SW_L_4 = old_df[:,7]

    FL_TOT_L_1 = np.concatenate((FL_ST_L_1, FL_SW_L_1),axis=0)
    FL_TOT_L_2 = np.concatenate((FL_ST_L_2, FL_SW_L_2),axis=0)
    FL_TOT_L_3 = np.concatenate((FL_ST_L_3, FL_SW_L_3),axis=0)
    FL_TOT_L_4 = np.concatenate((FL_ST_L_4, FL_SW_L_4),axis=0)

    HL_ST_R_1 = old_df[:,8]
    HL_ST_R_2 = old_df[:,9]
    HL_ST_R_3 = old_df[:,10]
    HL_ST_R_4 = old_df[:,11]

    HL_SW_R_1 = old_df[:,12]
    HL_SW_R_2 = old_df[:,13]
    HL_SW_R_3 = old_df[:,14]
    HL_SW_R_4 = old_df[:,15]

    HL_TOT_R_1 = np.concatenate((HL_ST_R_1, HL_SW_R_1),axis=0)
    HL_TOT_R_2 = np.concatenate((HL_ST_R_2, HL_SW_R_2),axis=0)
    HL_TOT_R_3 = np.concatenate((HL_ST_R_3, HL_SW_R_3),axis=0)
    HL_TOT_R_4 = np.concatenate((HL_ST_R_4, HL_SW_R_4),axis=0)

    HL_ST_L_1 = old_df[:,8]
    HL_ST_L_2 = old_df[:,9]
    HL_ST_L_3 = old_df[:,10]
    HL_ST_L_4 = old_df[:,11]

    HL_SW_L_1 = old_df[:,12]
    HL_SW_L_2 = old_df[:,13]
    HL_SW_L_3 = old_df[:,14]
    HL_SW_L_4 = old_df[:,15]

    HL_TOT_L_1 = np.concatenate((HL_ST_L_1, HL_SW_L_1),axis=0)
    HL_TOT_L_2 = np.concatenate((HL_ST_L_2, HL_SW_L_2),axis=0)
    HL_TOT_L_3 = np.concatenate((HL_ST_L_3, HL_SW_L_3),axis=0)
    HL_TOT_L_4 = np.concatenate((HL_ST_L_4, HL_SW_L_4),axis=0)

    dyn_con = 4096/360

    j = 0
    for i in FL_TOT_R_1:
        FL_TOT_R_1[j] = 2048 - (i * dyn_con)
        j += 1
    FL_TOT_R_1 = FL_TOT_R_1.round()
    FL_TOT_R_1 = list(map(int,FL_TOT_R_1))
    j = 0
    for i in FL_TOT_R_2:
        FL_TOT_R_2[j] = 2048 + (i * dyn_con)
        j += 1
    FL_TOT_R_2 = FL_TOT_R_2.round()
    FL_TOT_R_2 = list(map(int,FL_TOT_R_2))
    j = 0
    for i in FL_TOT_R_3:
        FL_TOT_R_3[j] = 2048 + (i * dyn_con)
        j += 1
    FL_TOT_R_3 = FL_TOT_R_3.round()
    FL_TOT_R_3 = list(map(int,FL_TOT_R_3))
    j = 0
    for i in FL_TOT_R_4:
        FL_TOT_R_4[j] = 2048 + (i * dyn_con)
        j += 1
    FL_TOT_R_4 = FL_TOT_R_4.round()
    FL_TOT_R_4 = list(map(int,FL_TOT_R_4))
    j = 0
    for i in FL_TOT_L_1:
        FL_TOT_L_1[j] = 2048 - (-1 * i * dyn_con)
        j += 1
    FL_TOT_L_1 = FL_TOT_L_1.round()
    FL_TOT_L_1 = list(map(int,FL_TOT_L_1))
    j = 0
    for i in FL_TOT_L_2:
        FL_TOT_L_2[j] = 2048 + (i * dyn_con)
        j += 1
    FL_TOT_L_2 = FL_TOT_L_2.round()
    FL_TOT_L_2 = list(map(int,FL_TOT_L_2))
    j = 0
    for i in FL_TOT_L_3:
        FL_TOT_L_3[j] = 2048 + (-1 * i * dyn_con)
        j += 1
    FL_TOT_L_3 = FL_TOT_L_3.round()
    FL_TOT_L_3 = list(map(int,FL_TOT_L_3))
    j = 0
    for i in FL_TOT_L_4:
        FL_TOT_L_4[j] = 2048 + (i * dyn_con)
        j += 1
    FL_TOT_L_4 = FL_TOT_L_4.round()
    FL_TOT_L_4 = list(map(int,FL_TOT_L_4))
    j = 0
    for i in HL_TOT_R_1:
        HL_TOT_R_1[j] = 2048 - (i * dyn_con)
        j += 1
    HL_TOT_R_1 = HL_TOT_R_1.round()
    HL_TOT_R_1 = list(map(int,HL_TOT_R_1))
    j = 0
    for i in HL_TOT_R_2:
        HL_TOT_R_2[j] = 2048 + (i * dyn_con)
        j += 1
    HL_TOT_R_2 = HL_TOT_R_2.round()
    HL_TOT_R_2 = list(map(int,HL_TOT_R_2))
    j = 0
    for i in HL_TOT_R_3:
        HL_TOT_R_3[j] = 2048 + (i * dyn_con)
        j += 1
    HL_TOT_R_3 = HL_TOT_R_3.round()
    HL_TOT_R_3 = list(map(int,HL_TOT_R_3))
    j = 0
    for i in HL_TOT_R_4:
        HL_TOT_R_4[j] = 2048 + (i * dyn_con)
        j += 1
    HL_TOT_R_4 = HL_TOT_R_4.round()
    HL_TOT_R_4 = list(map(int,HL_TOT_R_4))
    j = 0
    for i in HL_TOT_L_1:
        HL_TOT_L_1[j] = 2048 - (-1 * i * dyn_con)
        j += 1
    HL_TOT_L_1 = HL_TOT_L_1.round()
    HL_TOT_L_1 = list(map(int,HL_TOT_L_1))
    j = 0
    for i in HL_TOT_L_2:
        HL_TOT_L_2[j] = 2048 + (i * dyn_con)
        j += 1
    HL_TOT_L_2 = HL_TOT_L_2.round()
    HL_TOT_L_2 = list(map(int,HL_TOT_L_2))
    j = 0
    for i in HL_TOT_L_3:
        HL_TOT_L_3[j] = 2048 + (-1 * i * dyn_con)
        j += 1
    HL_TOT_L_3 = HL_TOT_L_3.round()
    HL_TOT_L_3 = list(map(int,HL_TOT_L_3))
    j = 0
    for i in HL_TOT_L_4:
        HL_TOT_L_4[j] = 2048 + (i * dyn_con)
        j += 1
    HL_TOT_L_4 = HL_TOT_L_4.round()
    HL_TOT_L_4 = list(map(int,HL_TOT_L_4))

    return FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4, HL_TOT_R_1, HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, HL_TOT_L_3, HL_TOT_L_4

def PostProcessPositions(FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, 
                        FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4, HL_TOT_R_1, 
                        HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, 
                        HL_TOT_L_3, HL_TOT_L_4):
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

    # ServoPos1 = ServoPos1.reshape(22,1)
    # ServoPos2 = ServoPos2.reshape(22,1)
    # ServoPos3 = ServoPos3.reshape(22,1)
    # ServoPos4 = ServoPos4.reshape(22,1)

    # ServoPos5 = ServoPos5.reshape(22,1)
    # ServoPos6 = ServoPos6.reshape(22,1)
    # ServoPos7 = ServoPos7.reshape(22,1)
    # ServoPos8 = ServoPos8.reshape(22,1)

    # ServoPos9 = ServoPos9.reshape(22,1)
    # ServoPos10 = ServoPos10.reshape(22,1)
    # ServoPos11 = ServoPos11.reshape(22,1)
    # ServoPos12 = ServoPos12.reshape(22,1)

    # ServoPos13 = ServoPos13.reshape(22,1)
    # ServoPos14 = ServoPos14.reshape(22,1)
    # ServoPos15 = ServoPos15.reshape(22,1)
    # ServoPos16 = ServoPos16.reshape(22,1)

    # FL_TOT = np.concatenate((ServoPos1, ServoPos2, ServoPos3, ServoPos4, ServoPos5, ServoPos6, ServoPos7, ServoPos8),axis=1)
    # HL_TOT = np.concatenate((ServoPos9, ServoPos10, ServoPos11, ServoPos12, ServoPos13, ServoPos14, ServoPos15, ServoPos16),axis=1)

    return [ServoPos1, ServoPos2, ServoPos3, ServoPos4, ServoPos5, ServoPos6, ServoPos7, ServoPos8, ServoPos9, ServoPos10, ServoPos11, ServoPos12, ServoPos13, ServoPos14, ServoPos15, ServoPos16]

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
    portHandler = PortHandler(DEVICENAME_1)

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
        return


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        return

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
    
def CleanUp(BodyObj,LimbObjList,ServoObjList):
    for each_servo in ServoObjList:
        each_servo.ToggleTorque(0,each_servo.portHandler,each_servo.packetHandler)
    BodyObj.__del__()
    for each_limb in LimbObjList:
        each_limb.__del__()
    for each_servo in ServoObjList:
        each_servo.__del__()

def ShutDown():
    print("Shutting down system.\n")
    # Turn off power to external boards and other systems
    print("Thank you for using Theo!")

def SpeedMerge(PositionsMatrix):
    desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
    TotMatrix_speeds = DetermineSpeeds(desired_timespan,PositionsMatrix)
    return TotMatrix_speeds

def AssembleRobot(PositionsArray):
    ServoObjList = []
    ServoObjDict = {}
    dxl_data_list = PingServos()
    for dxl_id in dxl_data_list:
        print("[ID:%03d] Detected" % (dxl_id))
        RelativeServo = Servo(dxl_id,PositionsArray[dxl_id-1][:])
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
    print("\n")
    for ServoID in ServoObjDict:
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
            FR_limbCount = 0
        if (FL_limbCount == 4):
            FL_Limb = [ServoObjDict[5],ServoObjDict[6],ServoObjDict[7],ServoObjDict[8]]
            FL_Leg = Leg(2,FL_Limb)
            TheoLimbList.append(FL_Leg)
            TheoLimbDict[2] = FL_Leg
            print("Front Left Limb is Digitally Assembled.")
            FL_limbCount = 0
        if (BR_limbCount == 4):
            BR_Limb = [ServoObjDict[9],ServoObjDict[10],ServoObjDict[11],ServoObjDict[12]]
            BR_Leg = Leg(3,BR_Limb)
            TheoLimbList.append(BR_Leg)
            TheoLimbDict[3] = BR_Leg
            print("Back Right Limb is Digitally Assembled.")
            BR_limbCount = 0
        if (BL_limbCount == 4):
            BL_Limb = [ServoObjDict[13],ServoObjDict[14],ServoObjDict[15],ServoObjDict[16]]
            BL_Leg = Leg(4,BL_Limb)
            TheoLimbList.append(BL_Leg)
            TheoLimbDict[4] = BL_Leg
            print("Back Left Limb is Digitally Assembled.")
            BL_limbCount = 0
        if (Neck_limbCount == 2):
            Neck_Limb = [ServoObjDict[17],ServoObjDict[18]]
            NeckStructure = Neck(5,Neck_Limb)
            TheoLimbList.append(NeckStructure)
            TheoLimbDict[5] = NeckStructure
            print("Neck Limb is Digitally Assembled.")
            Neck_limbCount = 0
        if (Spine_limbCount == 4):
            Spine_Limb = [ServoObjDict[19],ServoObjDict[20],ServoObjDict[21],ServoObjDict[22]]
            SpineStructure = Spine(6,Spine_Limb)
            TheoLimbList.append(SpineStructure)
            TheoLimbDict[6] = SpineStructure
            print("Spine Limb is Digitally Assembled.")
            Spine_limbCount = 0
        if (Tail_limbCount == 2):
            Tail_Limb = [ServoObjDict[23],ServoObjDict[24]]
            TailStructure = Tail(7,Tail_Limb)
            TheoLimbList.append(TailStructure)
            TheoLimbDict[7] = TailStructure
            print("Tail Limb is Digitally Assembled.")
            Tail_limbCount = 0

    TheoBody = Body(TheoLimbList)
    print("Body Structure Digitally Assembled.\n")

    return ServoObjList, ServoObjDict, TheoLimbList, TheoLimbDict, TheoBody

def RunThreads(ObjToMove1,portHandler1,portHandler2,packetHandler):
    global stopVal
    global t1
    global t2
    global thread_running
    t1 = Thread(target=ObjToMove1.ContinuousLegsMove,args=(portHandler1,portHandler2,packetHandler))
    #t2 = Thread(target=ObjToMove2.ContinuousMove,args=(portHandler,packetHandler))
    #t3 = Thread(target=ObjToMove3.ContinuousMove,args=(portHandler,packetHandler))
    #t4 = Thread(target=ObjToMove4.ContinuousMove,args=(portHandler,packetHandler))
    t5 = Thread(target=DetectStopInput)
    thread_running = True
    t1.start()
    #t2.start()
    t5.start()
    t5.join()
    thread_running = False
    stopVal = 0
