import time
from RelevantFunctions import *
import os
from ControlTable import *
from dynamixel_sdk import *
from time import sleep
import dynamixel_functions as dynamixel  
import csv

stopVal = 0

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
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch() # pylint: disable=undefined-variable
            quit()

        self.Positions = Positions
        self.Speeds = []
        self.HomeSpeed = STRAIGHT_SPEED
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
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_PROFILE_VELOCITY,InVelocity)
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
            quit()


        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

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
            quit()
        
        
        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
    
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

    def ContinuousMove(self,portHandler,packetHandler):
        global stopVal

        index = 1
        StartTime = time.perf_counter()
        firstMove = True
        while 1:
            self.SetServoVelocity(self.Speeds[index])
            self.MoveServo(self.Positions[index])
            self.UpdateOtherValues(self.Positions[index],self.Speeds[index])
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
                self.StrideIndex += 1
                StrideTimer = time.perf_counter()
                PhaseTimer = time.perf_counter()
            elif (index == 11):
                PhaseTimer = time.perf_counter()
            isStopped = 0
            while 1:
                dxl_mov, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, self.ID, ADDR_MOVING)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                if dxl_mov == 1:
                    pass
                else:
                    timeMarker = time.perf_counter()
                    isStopped == 1
                    break
                if isStopped == 0:
                    pass
                else:
                    self.UpdateTimeValues(index-1,StartTime,StrideTimer,PhaseTimer)
                    break

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
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
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
        print("[ID:%03d] Try factoryreset : " % (self.ID))
        dynamixel.factoryReset(port_num, PROTOCOL_VERSION, self.ID, OPERATION_MODE)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            print("Aborted")
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
            quit()
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
            quit()

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
            quit()

        sleep(0.2)

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

        self.ServoList = [servolist]

        getNum = lambda a : a.ID
        self.IDList = list(map(getNum,self.ServoList))

        listLen = len(self.IDList)
        rangeList = list(range(0,listLen))
        self.ServoDict = {}

        for i in rangeList:
            self.ServoDict[i] = self.IDList[i] 

        self.IsHome = None # Check all Servo Members to set this value

        self.TotalTime = 0

        self.GoalVelocity = []
        self.GoalPosition = []





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

    def ReadData(self,IndexIn,portHandler,packetHandler):
        # Initialize GroupSyncRead instace for Present Position
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        for h in self.IDList:
            dxl_addparam_result = groupSyncRead.addParam(h)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % h)
                quit()

        PresentPositions = []

        for e in self.IDList:
            # Check if groupsyncread data of Dynamixel is available
            dxl_getdata_result = groupSyncRead.isAvailable(e, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % e)
                quit()

            # Get Dynamixel present position value

        dxl_present_position = groupSyncRead.getData(e, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        PresentPositions.append(dxl_present_position)

        print("======================================================================")
        GetIndexString = f"Stride #{self.StrideCount}, Movement #{IndexIn}"
        print(GetIndexString)
        for f,g in self.ServoDict:
            GetPosString = "[ID:%03d] GoalPos:%03d  PresPos:%03d" % (g, self.GoalPosition[f], PresentPositions[f])
            GetVelString = "[ID:%03d] Velocity Given:%03d" % (g, self.GoalVelocity[f])
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

        for b in self.ServoList:
            self.GoalVelocity.append(FormatSendData(b.Speeds[IndexIn]))
            self.GoalPosition.append(FormatSendData(b.Positions[IndexIn]))

        for c, d in self.ServoDict:
            dxl_addparam_result = groupSyncWriteVEL.addParam(d,self.GoalVelocity[c])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d)
                quit()


            dxl_addparam_result = groupSyncWritePOS.addParam(d,self.GoalPosition[c])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % d)
                quit()

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

        if (ReadOption == True):
            self.ReadData(IndexIn,portHandler,packetHandler)
        else:
            pass

    def MoveHome(self,portHandler,packetHandler,ReadOption=False):
        self.MoveLimb(0,portHandler,packetHandler,ReadOption)
        self.IsHome = True
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ContinuousMove(self,portHandler,packetHandler):
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

                for i,j in self.ServoDict:
                    # Get Dynamixel#1 present Moving value
                    dxl_mov = groupSyncRead.getData(j, ADDR_MOVING, LEN_MOVING)
                    if (dxl_mov == 0) and (isStopped[i] == 0):
                        timeMarkers[i] = time.perf_counter()
                        isStopped[i] = 1

                if 0 in isStopped:
                    pass
                else:
                    self.UpdateTimeValues(index-1,StartTime,StrideTimer,PhaseTimer)
                    # PRINT INFORMATION TO DOCUMENT HERE USING TIMEMARKERS LIST
                    break

    def UpdateTimeValues(self,IndexIn,StartTimeRef,StartStrideRef,StartPhaseRef):
        RelevReferenceTime = time.perf_counter()
        if (IndexIn == 0):
            self.IsHome = True
        else:
            self.IsHome = False
        self.PhaseTime = RelevReferenceTime - StartPhaseRef
        self.StrideTime = RelevReferenceTime - StartStrideRef
        self.TotalTime = RelevReferenceTime - StartTimeRef


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


class DataDocument:
    def __init__(self):
        self.Directory = None
        self.Name = None
        self.FileExtension = ".txt"
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
        self.FileRef = open(self.CompleteName,self.FilePermission)
        self.FileExists = 1

    def CheckForDoc(self):
        if self.FileExists == 0:
            return False
        else:
            return True

    def WriteToDoc(self,InData):
        self.FilePermission = 'a+' # Need to close and open with different permission?
        self.FileRef.write(InData)

    def CloseDoc(self):
        self.FilePermission = 'a+'
        self.FileRef.close()
        self.FileExists = 0

    def __del__(self):
        pass