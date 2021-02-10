import os
from ControlTable import *
from dynamixel_sdk import *
import numpy as np
import copy as cp
import time
from curtsies import Input
from threading import Thread
import RPi.GPIO as GPIO
#import dynamixel_functions as dynamixel  
import csv

stopVal = 0
global t1
global t2

################################ CLASS SECTION ####################################################
class Servo:
    def __init__(self,IDnum,Positions):
        self.ID = IDnum
        
        self.portHandler = PortHandler(DEVICENAME)

        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch() # pylint: disable=undefined-variable
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch() # pylint: disable=undefined-variable
            return

        self.Positions = Positions
        self.Speeds = []
        self.HomeSpeed = STRAIGHT_SPEED
        self.IsHome = None
        self.Activated = 1 # Change this to zero if you don't want it to move
        self.FirstMovePosition = 0
        self.OffsetPercent = 0
        self.Phase = 0 # 2 for Stance, 1 for transition to stance, -2 for swing, -1 for transition to swing, 0 for other
        self.PresentPosition = 0
        self.GivenPosition = 0
        self.GivenSpeed = 0
        self.MovementTime = 0 # Length of time for the last movement given
        self.PhaseIndex = 0 
        self.StrideIndex = 1
        self.PhaseTime = 0
        self.StrideTime = 0
        self.TotalTime = 0
        self.IndexShifts = RotatePositionArray(list(range(0,22)),self.OffsetPercent/10,len(list(range(0,22))))
        self.DataArray = []

    def InitialSetup(self): 
        #Set drive mode to velocity based
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, ADDR_DRIVE_MODE, DRIVE_MODE_VEL_BASED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Drive mode set to: %03d" %(self.ID, DRIVE_MODE_VEL_BASED))

        time.sleep(PreferedDelay)

        #Set operating mode to joint/position control mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, ADDR_OPERATING_MODE, OPERATING_JOINT_POSITION_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Operating mode set to: %03d" %(self.ID, OPERATING_JOINT_POSITION_MODE))

        time.sleep(PreferedDelay)

        #Set acceleration limit 
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_ACCELERATION_LIMIT, ACCELERATION_LIMIT_M)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Acceleration limit set to: %03d" %(self.ID, ACCELERATION_LIMIT_M))

        time.sleep(PreferedDelay)

        #Set max position limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Max position limit set to: %03d" %(self.ID, MAX_POSITION_LIMIT))

        time.sleep(PreferedDelay)

        #Set min position limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Min position limit set to: %03d" %(self.ID, MIN_POSITION_LIMIT))

        time.sleep(PreferedDelay)

        #Set moving threshold
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD_ACCURACY_H)
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

    def SetServoVelocity(self,InVelocity):
        print(InVelocity)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_PROFILE_VELOCITY,int(InVelocity))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Velocity limit set to: %03d" %(self.ID, InVelocity))

    def MoveServo(self,InPosition):
         # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_PRO_GOAL_POSITION, InPosition)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Goal Position set to: %03d" %(self.ID, InPosition))

    def MoveHome(self):
        self.SetServoVelocity(self.HomeSpeed)
        self.MoveServo(self.Positions[0])

    def ReadTrait(self,traitNum):
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
        portHandler = self.portHandler

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packetHandler = self.packetHandler

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

        DataAddr = DataAddrConversion(traitNum)

        if (traitNum == 1):
            ModelNumber, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Model Number: %03d" % (self.ID, ModelNumber))
        elif (traitNum == 2):
            ModelInfo, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Model Information: %03d" % (self.ID, ModelInfo))
        elif (traitNum == 3):
            FirmwareVer, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Firmware Version: %03d" % (self.ID, FirmwareVer))
        elif (traitNum == 4):
            IDnum, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] ID Number: %03d" % (self.ID, IDnum))
        elif (traitNum == 5):
            BaudRate, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Baud Rate: %03d" % (self.ID, BaudRate))
        elif (traitNum == 6):
            RetDelayTime, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Return Delay Time: %03d" % (self.ID, RetDelayTime))
        elif (traitNum == 7):
            DriveMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Drive Mode: %03d" % (self.ID, DriveMode))
        elif (traitNum == 8):
            OperateMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Operating Mode: %03d" % (self.ID, OperateMode))
        elif (traitNum == 9):
            ProtocType, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Protocol Type: %03d" % (self.ID, ProtocType))
        elif (traitNum == 10):
            HomeOffset, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Homing Offset: %03d" % (self.ID, HomeOffset))
        elif (traitNum == 11):
            MoveThreshold, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Threshold: %03d" % (self.ID, MoveThreshold))
        elif (traitNum == 12):
            TempLimit, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Temperature Limit: %03d" % (self.ID, TempLimit))
        elif (traitNum == 13):
            MaxVoltLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Max Voltage Limit: %03d" % (self.ID, MaxVoltLimit))
        elif (traitNum == 14):
            MinVoltLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Min Voltage Limit: %03d" % (self.ID, MinVoltLimit))
        elif (traitNum == 15):
            PWMlimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] PWM Limit: %03d" % (self.ID, PWMlimit))
        elif (traitNum == 16):
            CurrLimit, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Current Limit: %03d" % (self.ID, CurrLimit))
        elif (traitNum == 17):
            AccLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Acceleration Limit: %03d" % (self.ID, AccLimit))
        elif (traitNum == 18):
            VelLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity Limit: %03d" % (self.ID, VelLimit))
        elif (traitNum == 19):
            MaxPosLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Max Position Limit: %03d" % (self.ID, MaxPosLimit))
        elif (traitNum == 20):
            MinPosLimit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Min Position Limit: %03d" % (self.ID, MinPosLimit))
        elif (traitNum == 21):
            ShutdownVal, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Shutdown Value: %03d" % (self.ID, ShutdownVal))
        elif (traitNum == 22):
            TorqToggle, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Torque Toggle Value: %03d" % (self.ID, TorqToggle))
        elif (traitNum == 23):
            LEDtoggle, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] LED Toggle Value: %03d" % (self.ID, LEDtoggle))
        elif (traitNum == 24):
            StatusRetLevel, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Status Return Level: %03d" % (self.ID, StatusRetLevel))
        elif (traitNum == 25):
            RegInstruction, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Registered Instruction: %03d" % (self.ID, RegInstruction))
        elif (traitNum == 26):
            HardErrStat, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Hardware Error Status: %03d" % (self.ID, HardErrStat))
        elif (traitNum == 27):
            VelIgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity I Gain: %03d" % (self.ID, VelIgain))
        elif (traitNum == 28):
            VelPgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity P Gain: %03d" % (self.ID, VelPgain))
        elif (traitNum == 29):
            PosDgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position D Gain: %03d" % (self.ID, PosDgain))
        elif (traitNum == 30):
            PosIgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position I Gain: %03d" % (self.ID, PosIgain))
        elif (traitNum == 31):
            PosPgain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position P Gain: %03d" % (self.ID, PosPgain))
        elif (traitNum == 32):
            GoalPWM, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal PWM: %03d" % (self.ID, GoalPWM))
        elif (traitNum == 33):
            GoalCurr, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Current: %03d" % (self.ID, GoalCurr))
        elif (traitNum == 34):
            GoalVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Velocity: %03d" % (self.ID, GoalVel))
        elif (traitNum == 35):
            ProfAccel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Profile Acceleration: %03d" % (self.ID, ProfAccel))
        elif (traitNum == 36):
            ProfVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Profile Velocity: %03d" % (self.ID, ProfVel))
        elif (traitNum == 37):
            GoalPos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Position: %03d" % (self.ID, GoalPos))
        elif (traitNum == 38):
            RealtimeTick, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Realtime Tick: %03d" % (self.ID, RealtimeTick))
        elif (traitNum == 39):
            MovingVal, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Value: %03d" % (self.ID, MovingVal))
        elif (traitNum == 40):
            MovingStat, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Status: %03d" % (self.ID, MovingStat))
        elif (traitNum == 41):
            PresentPWM, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present PWM: %03d" % (self.ID, PresentPWM))
        elif (traitNum == 42):
            PresentCurr, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Current: %03d" % (self.ID, PresentCurr))
        elif (traitNum == 43):
            PresentVel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Velocity: %03d" % (self.ID, PresentVel))
        elif (traitNum == 44):
            PresentPos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Position: %03d" % (self.ID, PresentPos))
        elif (traitNum == 45):
            VelTraj, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity Trajectory: %03d" % (self.ID, VelTraj))
        elif (traitNum == 46):
            PosTraj, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position Trajectory: %03d" % (self.ID, PosTraj))
        elif (traitNum == 47):
            PresInVoltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Input Voltage: %03d" % (self.ID, PresInVoltage))
        elif (traitNum == 48):
            PresTemp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, DataAddr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Temperature: %03d" % (self.ID, PresTemp))
        else:
            print("Data does not have a matchable address")
        # Close port
        portHandler.closePort()

    def WriteTrait(self,traitNum,traitValue):
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
        portHandler = self.portHandler
    
        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packetHandler = self.packetHandler
    
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
    
        DataAddr = DataAddrConversion(traitNum)
    
        if (traitNum == 1):
            ModelNumber = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, ModelNumber)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Model Number Changed To: %03d" % (self.ID, ModelNumber))
        elif (traitNum == 2):
            ModelInfo = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, ModelInfo)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Model Information Changed To: %03d" % (self.ID, ModelInfo))
        elif (traitNum == 3):
            FirmwareVer = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, FirmwareVer)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Firmware Version Changed To: %03d" % (self.ID, FirmwareVer))
        elif (traitNum == 4):
            IDnum = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, IDnum)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] ID Number Changed To: %03d" % (self.ID, IDnum))
        elif (traitNum == 5):
            BaudRate = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, BaudRate)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Baud Rate Changed To: %03d" % (self.ID, BaudRate))
        elif (traitNum == 6):
            RetDelayTime = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, RetDelayTime)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Return Delay Time Changed To: %03d" % (self.ID, RetDelayTime))
        elif (traitNum == 7):
            DriveMode = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, DriveMode)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Drive Mode Changed To: %03d" % (self.ID, DriveMode))
        elif (traitNum == 8):
            OperateMode = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, OperateMode)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Operating Mode Changed To: %03d" % (self.ID, OperateMode))
        elif (traitNum == 9):
            ProtocType = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, ProtocType)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Protocol Type Changed To: %03d" % (self.ID, ProtocType))
        elif (traitNum == 10):
            HomeOffset = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, HomeOffset)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Homing Offset Changed To: %03d" % (self.ID, HomeOffset))
        elif (traitNum == 11):
            MoveThreshold = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, MoveThreshold)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Threshold Changed To: %03d" % (self.ID, MoveThreshold))
        elif (traitNum == 12):
            TempLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, TempLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Temperature Limit Changed To: %03d" % (self.ID, TempLimit))
        elif (traitNum == 13):
            MaxVoltLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, MaxVoltLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Max Voltage Limit Changed To: %03d" % (self.ID, MaxVoltLimit))
        elif (traitNum == 14):
            MinVoltLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, MinVoltLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Min Voltage Limit Changed To: %03d" % (self.ID, MinVoltLimit))
        elif (traitNum == 15):
            PWMlimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PWMlimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] PWM Limit Changed To: %03d" % (self.ID, PWMlimit))
        elif (traitNum == 16):
            CurrLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, CurrLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Current Limit Changed To: %03d" % (self.ID, CurrLimit))
        elif (traitNum == 17):
            AccLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, AccLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Acceleration Limit Changed To: %03d" % (self.ID, AccLimit))
        elif (traitNum == 18):
            VelLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, VelLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity Limit Changed To: %03d" % (self.ID, VelLimit))
        elif (traitNum == 19):
            MaxPosLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, MaxPosLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Max Position Limit Changed To: %03d" % (self.ID, MaxPosLimit))
        elif (traitNum == 20):
            MinPosLimit = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, MinPosLimit)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Min Position Limit Changed To: %03d" % (self.ID, MinPosLimit))
        elif (traitNum == 21):
            ShutdownVal = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, ShutdownVal)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Shutdown Value Changed To: %03d" % (self.ID, ShutdownVal))
        elif (traitNum == 22):
            TorqToggle = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, TorqToggle)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Torque Toggle Value Changed To: %03d" % (self.ID, TorqToggle))
        elif (traitNum == 23):
            LEDtoggle = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, LEDtoggle)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] LED Toggle Value Changed To: %03d" % (self.ID, LEDtoggle))
        elif (traitNum == 24):
            StatusRetLevel = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, StatusRetLevel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Status Return Level Changed To: %03d" % (self.ID, StatusRetLevel))
        elif (traitNum == 25):
            RegInstruction = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, RegInstruction)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Registered Instruction Changed To: %03d" % (self.ID, RegInstruction))
        elif (traitNum == 26):
            HardErrStat = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, HardErrStat)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Hardware Error Status Changed To: %03d" % (self.ID, HardErrStat))
        elif (traitNum == 27):
            VelIgain = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, VelIgain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity I Gain Changed To: %03d" % (self.ID, VelIgain))
        elif (traitNum == 28):
            VelPgain = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, VelPgain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity P Gain Changed To: %03d" % (self.ID, VelPgain))
        elif (traitNum == 29):
            PosDgain = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PosDgain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position D Gain Changed To: %03d" % (self.ID, PosDgain))
        elif (traitNum == 30):
            PosIgain = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PosIgain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position I Gain Changed To: %03d" % (self.ID, PosIgain))
        elif (traitNum == 31):
            PosPgain = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PosPgain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position P Gain Changed To: %03d" % (self.ID, PosPgain))
        elif (traitNum == 32):
            GoalPWM = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, GoalPWM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal PWM Changed To: %03d" % (self.ID, GoalPWM))
        elif (traitNum == 33):
            GoalCurr = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, GoalCurr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Current Changed To: %03d" % (self.ID, GoalCurr))
        elif (traitNum == 34):
            GoalVel = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, GoalVel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Velocity Changed To: %03d" % (self.ID, GoalVel))
        elif (traitNum == 35):
            ProfAccel = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, ProfAccel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Profile Acceleration Changed To: %03d" % (self.ID, ProfAccel))
        elif (traitNum == 36):
            ProfVel = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, ProfVel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Profile Velocity Changed To: %03d" % (self.ID, ProfVel))
        elif (traitNum == 37):
            GoalPos = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, GoalPos)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Goal Position Changed To: %03d" % (self.ID, GoalPos))
        elif (traitNum == 38):
            RealtimeTick = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, RealtimeTick)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Realtime Tick Changed To: %03d" % (self.ID, RealtimeTick))
        elif (traitNum == 39):
            MovingVal = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, MovingVal)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Value Changed To: %03d" % (self.ID, MovingVal))
        elif (traitNum == 40):
            MovingStat = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, MovingStat)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Moving Status Changed To: %03d" % (self.ID, MovingStat))
        elif (traitNum == 41):
            PresentPWM = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PresentPWM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present PWM Changed To: %03d" % (self.ID, PresentPWM))
        elif (traitNum == 42):
            PresentCurr = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PresentCurr)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Current Changed To: %03d" % (self.ID, PresentPWM))
        elif (traitNum == 43):
            PresentVel = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, PresentVel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Velocity Changed To: %03d" % (self.ID, PresentVel))
        elif (traitNum == 44):
            PresentPos = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, PresentPos)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Position Changed To: %03d" % (self.ID, PresentPos))
        elif (traitNum == 45):
            VelTraj = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, VelTraj)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Velocity Trajectory Changed To: %03d" % (self.ID, VelTraj))
        elif (traitNum == 46):
            PosTraj = traitValue
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.ID, DataAddr, PosTraj)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Position Trajectory Changed To: %03d" % (self.ID, PosTraj))
        elif (traitNum == 47):
            PresInVoltage = traitValue
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.ID, DataAddr, PresInVoltage)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Input Voltage Changed To: %03d" % (self.ID, PresInVoltage))
        elif (traitNum == 48):
            PresTemp = traitValue
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, DataAddr, PresTemp)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("[ID:%03d] Present Temperature Changed To: %03d" % (self.ID, PresTemp))
        else:
            print("Data does not have a matchable address")
    
        # Close port
        portHandler.closePort()

    def ContinuousMove(self,portHandler,packetHandler,DataRecord,CurrentDoc):
        global stopVal
        index = 1
        StartTime = time.perf_counter()
        firstMove = True
        print("AfterSpeeds")
        print(np.size(self.Speeds,0))
        while 1:
            if (index > 21):
                index = 0
            elif (index == 1):
                self.StrideIndex += 1
                StrideTimer = time.perf_counter()
                PhaseTimer = time.perf_counter()
            elif (index == 11):
                PhaseTimer = time.perf_counter()
                
            self.SetServoVelocity(self.Speeds[index])
            self.MoveServo(self.Positions[index])
            
            if (firstMove == True):
                StrideTimer = time.perf_counter()
                PhaseTimer = time.perf_counter()
                firstMove == False
            #index += 1
            if (stopVal == 1):
                print("\nFinishing Movement.\n")
                break
            while 1:
                dxl_mov, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, ADDR_MOVING)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                if dxl_mov == 1:
                    pass
                else:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.ID, ADDR_PRO_PRESENT_POSITION)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    timeMarker = time.perf_counter()
                    self.UpdatePhase(index)
                    self.UpdateTimeValues(index-1,StartTime,StrideTimer,PhaseTimer)
                    self.UpdateOtherValues(self.Positions[index],self.Speeds[index]) 
                    if DataRecord == True:
                        CurrentDoc.WriteToDoc(self.DataArray)
                    else:
                        pass
                    break
            index += 1

    def DisplayTraitValues(self):
        print("Relevant Traits Will Print Out When Called...")

    def UpdatePhase(self,IndexIn):
        if self.IndexShifts[IndexIn] in list(range(1,11)):
            self.Phase = 2
        elif self.IndexShifts[IndexIn] == 11:
            self.Phase = -1
        elif self.IndexShifts[IndexIn] in list(range(12,22)):
            self.Phase = -2
        elif self.IndexShifts[IndexIn] == 0:
            self.Phase = 1
        else:
            self.Phase = 0

    def UpdateTimeValues(self,IndexIn,StartTimeRef,StartStrideRef,StartPhaseRef):
        RelevReferenceTime = time.perf_counter()
        if (IndexIn == 0):
            self.IsHome = True
        else:
            self.IsHome = False
        self.PhaseTime = RelevReferenceTime - StartPhaseRef
        self.StrideTime = RelevReferenceTime - StartStrideRef
        self.TotalTime = RelevReferenceTime - StartTimeRef

    def UpdateOtherValues(self,GivPos,GivVel):
        self.GivenPosition = GivPos
        self.GivenSpeed = GivVel
        self.DataArray = [self.ID,self.GivenPosition,self.PresentPosition,self.GivenSpeed,self.IsHome,self.FirstMovePosition,self.StrideIndex,self.Phase,self.PhaseTime,self.StrideTime,self.TotalTime]

    def RebootServo(self):
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

    def ResetServo(self):
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
        offsets = []
        for k in self.ServoList:
            offsets.append(k.OffsetPercent)
        if len(set(offsets))==1:
            self.Offset = offsets[0]
        else:
            print("The Offsets for the Servos that make up this Leg are not consistent. Please Fix.")
        self.IndexShifts = RotatePositionArray(list(range(0,22)),self.Offset/10,len(list(range(0,22))))
        self.DataArray1 = []
        self.DataArray2 = []
        self.DataArray3 = []
        self.DataArray4 = []

        self.OnceOrCont = 0

    def ReadData(self,IndexIn,portHandler,packetHandler):
        # Initialize GroupSyncRead instace for Present Position
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        for h in self.IDList:
            dxl_addparam_result = groupSyncRead.addParam(h)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % h)
                return

        PresentPositions = []

        for e in self.IDList:
            # Check if groupsyncread data of Dynamixel is available
            dxl_getdata_result = groupSyncRead.isAvailable(e, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % e)
                return

            # Get Dynamixel present position value
            else:
                dxl_present_position = groupSyncRead.getData(e, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                PresentPositions.append(dxl_present_position)

        print("======================================================================")
        GetIndexString = f"Stride #{self.StrideCount}, Movement #{IndexIn}"
        print(GetIndexString)
        for f,g in self.ServoDict.items():
            GetPosString = "[ID:%03d] GoalPos:%03d  PresPos:%03d" % (g, self.GoalPosition[f-1], PresentPositions[f-1])
            GetVelString = "[ID:%03d] Velocity Given:%03d" % (g, self.GoalVelocity[f-1])
            print(GetPosString)
            print(GetVelString)
        print("======================================================================")
        
        # Clear syncread parameter storage
        groupSyncRead.clearParam()

    def MoveLimb(self,IndexIn,portHandler,packetHandler,ReadOption=True):
         # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        self.GoalVelocity = []
        self.GoalPosition = []

        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(int(b.Speeds[IndexIn])))
            self.GoalPosition.append(FormatSendData(b.Positions[IndexIn]))

        index = 0
        for _ , d in self.ServoDict.items():
            FormattedVel = self.GoalVelocity[index]
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

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePOS.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVEL.clearParam()

        # Clear syncwrite parameter storage
        groupSyncWritePOS.clearParam()

        if (ReadOption == True):
            self.ReadData(IndexIn,portHandler,packetHandler)
        else:
            pass

    def MoveHome(self,portHandler,packetHandler,ReadOption=False):
        self.MoveLimb(0,portHandler,packetHandler,ReadOption)
        self.IsHome = True
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler,DataRecord,CurrentDoc):
        global stopVal
        # Initialize GroupSyncRead instace for Present Position
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        index = 1
        StartTime = time.perf_counter()
        firstMove = True
        while 1:
            self.MoveLimb(index,portHandler,packetHandler,ReadOption=True)
            if (firstMove == True):
                StrideTimer = time.perf_counter()
                PhaseTimer = time.perf_counter()
                firstMove == False
            index += 1
            if (stopVal == 1):
                print("\nFinishing Movement.\n")
                break
            if (index > 21):
                index = 0
            elif (index == 1):
                self.StrideCount += 1
                StrideTimer = time.perf_counter()
                PhaseTimer = time.perf_counter()
            elif (index == 11):
                PhaseTimer = time.perf_counter()
            isStopped = [0] * len(self.ServoList)
            timeMarkers = [0] * len(self.ServoList)
            while 1:
                # Syncread Moving Value
                dxl_comm_result = groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

                for i,j in self.ServoDict.items():
                    # Get Dynamixel#1 present Moving value
                    dxl_mov = groupSyncRead.getData(j, ADDR_MOVING, LEN_MOVING)
                    
                    if (dxl_mov == 0) and (isStopped[i-1] == 0):
                        timeMarkers[i-1] = time.perf_counter()
                        isStopped[i-1] = 1
                    #self.PresentPositions[i-1] = 

                if 0 in isStopped:
                    pass
                else:
                    self.UpdatePhase(index)
                    self.UpdateTimeValues(index-1,StartTime,StrideTimer,PhaseTimer)
                    self.UpdateOtherValues(index)
                    # PRINT INFORMATION TO DOCUMENT HERE USING TIMEMARKERS LIST
                    if DataRecord == True: 
                        CurrentDoc.WriteToDoc(self.DataArray1)
                        CurrentDoc.WriteToDoc(self.DataArray2)
                        CurrentDoc.WriteToDoc(self.DataArray3)
                        CurrentDoc.WriteToDoc(self.DataArray4)
                    break

                for h in self.IDList:
                    dxl_addparam_result = groupSyncRead.addParam(h)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncRead addparam failed" % h)
                    return


                # Syncread present position
                dxl_comm_result = groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

                        # Syncread present position
                dxl_comm_result = groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

                #time.sleep(PreferedDelay)


                for j in  self.IDList:
                    dxl_getdata_result = groupSyncRead.isAvailable(j, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                    if dxl_getdata_result != True:
                        print("[ID:%03d] groupSyncRead getdata failed" % j)

                        return



                #time.sleep(PreferedDelay)

                # Get Dynamixel#1 present position value
                dxl1_present_position = groupSyncRead.getData(self.IDList[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                #time.sleep(PreferedDelay)

                # Get Dynamixel#2 present position value
                dxl2_present_position = groupSyncRead.getData(self.IDList[1], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                #time.sleep(PreferedDelay)

                # Get Dynamixel#3 present position value
                dxl3_present_position = groupSyncRead.getData(self.IDList[2], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                #time.sleep(PreferedDelay)

                # Get Dynamixel#4 present position value
                dxl4_present_position = groupSyncRead.getData(self.IDList[3], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                self.PresentPositions = [dxl1_present_position,dxl2_present_position,dxl3_present_position,dxl4_present_position]


    def UpdatePhase(self,IndexIn):
        for k in self.ServoList:
            if self.IndexShifts[IndexIn] in list(range(1,11)):
                k.Phase = 2
            elif self.IndexShifts[IndexIn] == 11:
                k.Phase = -1
            elif self.IndexShifts[IndexIn] in list(range(12,22)):
                k.Phase = -2
            elif self.IndexShifts[IndexIn] == 0:
                k.Phase = 1
            else:
                k.Phase = 0
    def UpdateTimeValues(self,IndexIn,StartTimeRef,StartStrideRef,StartPhaseRef):
        RelevReferenceTime = time.perf_counter()
        if (IndexIn == 0):
            self.IsHome = True
            self.FirstMovePosition = False
        elif (IndexIn == 1):
            self.IsHome = False
            self.FirstMovePosition = True
        elif (IndexIn >= 0 and IndexIn <= 10):
            self.IsHome = False
            self.FirstMovePosition = False
        elif (IndexIn >= 11 and IndexIn <= 21):
            self.IsHome = False
            self.FirstMovePosition = False

        if (self.LimbNumber == 1):
             self.StrideIndex = IndexIn + 1
             if IndexIn == 21:
                 self.StrideIndex = 0   
        elif (self.LimbNumber == 2):
            self.StrideIndex = IndexIn + 9
            if IndexIn >= 13:
                self.StrideIndex = self.StrideIndex - 22
        elif (self.LimbNumber == 3):
            self.StrideIndex = IndexIn + 8
            if IndexIn >= 14:
                self.StrideIndex = self.StrideIndex = self.StrideIndex - 22
        elif (self.LimbNumber == 4):
            self.StrideIndex = IndexIn

        self.PhaseTime = RelevReferenceTime - StartPhaseRef
        self.StrideTime = RelevReferenceTime - StartStrideRef
        self.TotalTime = RelevReferenceTime - StartTimeRef
        
    def UpdateOtherValues(self,IndexIn):
        self.GoalVelocity = []
        self.GoalPosition = []

        for b in self.ServoList:
            self.GoalVelocity.append(b.Speeds[IndexIn])
            self.GoalPosition.append(b.Positions[IndexIn])

        self.DataArray1 = [self.IDList[0],self.GoalPosition[0],self.PresentPositions[0],self.GoalVelocity[0],self.IsHome,self.FirstMovePosition,self.StrideIndex,self.Phase,self.PhaseTime,self.StrideTime,self.TotalTime]
        self.DataArray2 = [self.IDList[1],self.GoalPosition[1],self.PresentPositions[1],self.GoalVelocity[1],self.IsHome,self.FirstMovePosition,self.StrideIndex,self.Phase,self.PhaseTime,self.StrideTime,self.TotalTime]
        self.DataArray3 = [self.IDList[2],self.GoalPosition[2],self.PresentPositions[2],self.GoalVelocity[2],self.IsHome,self.FirstMovePosition,self.StrideIndex,self.Phase,self.PhaseTime,self.StrideTime,self.TotalTime]
        self.DataArray4 = [self.IDList[3],self.GoalPosition[3],self.PresentPositions[3],self.GoalVelocity[3],self.IsHome,self.FirstMovePosition,self.StrideIndex,self.Phase,self.PhaseTime,self.StrideTime,self.TotalTime]

    def __del__(self):
            pass        


class Neck(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

class Spine(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

class Tail(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)



class Body:
    def __init__(self,limbs):
        self.limbs = [limbs]

    def MoveBody(self,IndexIn,portHandler,packetHandler):
        for SingleLimb in self.limbs:
            SingleLimb.MoveLimb(IndexIn,portHandler,PacketHandler,ReadOption=True)

    def MoveHome(self,portHandler,packetHandler):
        for SingleLimb in self.limbs:
            SingleLimb.MoveHome(portHandler,PacketHandler,ReadOption=False)

    def ContinuousMove(self,portHandler,packetHandler):
        pass # Going to need threads for these???

    def __del__(self):
            pass



class DataDocument:
    def __init__(self):
        self.Directory = None
        self.Name = None
        self.FileExtension = ".csv"
        self.FileExists = 0
        self.FilePermission = 'w+'
        self.FileRef = None
        self.CompleteName = None

    def CreateDoc(self):
        print("Your data file will be saved in the Records sub-folder.\n")
        tempName = input("Enter the name of the File you want to create: ")
        print("\n")
        self.Name = tempName + self.FileExtension
        path = os.getcwd()
        self.Directory = path + '\\Records'
        self.CompleteName = self.Directory + '\\' + self.Name
        headers = ["Servo ID","Goal Position","Position at time of Recording","Speed Given to get to goal position","Is This Home Position?",
           "Is this the first position it moves to after home position?","Index of Movement",
           "Phase is Stance or Swing or transition?","Time Value Within Phase","Time Value Within Stride",
           "Time Value Total"]
        with open(self.CompleteName, 'w', newline='') as csvfile:
            DocWriter = csv.writer(csvfile, delimiter=',')
            DocWriter.writerow(headers)
        self.FileExists = 1
            

    def CheckForDoc(self):
        if self.FileExists == 0:
            return False
        else:
            return True

    def WriteToDoc(self,InData):
        with open(self.CompleteName, 'a', newline='') as csvfile:
            DocWriter = csv.writer(csvfile, delimiter=',',quoting=csv.QUOTE_MINIMAL)
            DocWriter.writerow(InData)

    def __del__(self):
        pass

################################ FUNCTIONS SECTION ####################################################

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
        return


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        return

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
    print("\nShape of Speeds")
    print(np.shape(speeds))
    print("\n")
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
    TOT_VEL = np.concatenate((FL_VEL,HL_VEL),axis=1)
    print("\nShape of Vels")
    print(np.shape(TOT_VEL))
    print("\n")
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
    
def CleanUp(BodyObj,LimbObjList,ServoObjList,CurrentDoc):
    for each_servo in ServoObjList:
        each_servo.ToggleTorque(0,each_servo.portHandler,each_servo.packetHandler)
    BodyObj.__del__()
    for each_limb in LimbObjList:
        each_limb.__del__()
    for each_servo in ServoObjList:
        each_servo.__del__()
    if CurrentDoc != None:
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
        RelativeServo = Servo(dxl_id,PositionsArray[:,dxl_id])
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

def RunThreads(ObjToMove,portHandler,packetHandler,DataRecord,CurrentDoc):
    global stopVal
    global t1
    global t2
    global thread_running
    t1 = Thread(target=ObjToMove.ContinuousMove,args=(portHandler,packetHandler,DataRecord,CurrentDoc))
    t2 = Thread(target=DetectStopInput)
    thread_running = True
    t1.start()
    t2.start()
    t2.join()
    thread_running = False
    stopVal = 0

def PulsePin(PinNum):
    GPIO.output(PinNum, 1)
    time.sleep(PinPulseTime)
    GPIO.output(PinNum, 0)
