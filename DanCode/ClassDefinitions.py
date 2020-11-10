import time
import RelevantFunctions
import os
from ControlTable import *
from dynamixel_sdk import *

stopVal = 0

class Servo:
    def __init__(self,IDnum,Speeds,Positions):
        self.ID = IDnum

        if (IDnum == 1 or IDnum == 2 or IDnum == 3 or IDnum == 4):
            self.Limbnum = 1
        elif (IDnum == 5 or IDnum == 6 or IDnum == 7 or IDnum == 8):
            self.Limbnum = 2
        elif (IDnum == 9 or IDnum == 10 or IDnum == 11 or IDnum == 12):
            self.Limbnum = 3
        elif (IDnum == 13 or IDnum == 14 or IDnum == 15 or IDnum == 16):
            self.Limbnum = 4
        elif (IDnum == 17 or IDnum == 18):
            self.Limbnum = 5
        elif (IDnum == 19 or IDnum == 20 or IDnum == 21 or IDnum == 22):
            self.Limbnum = 6
        elif (IDnum == 23 or IDnum == 24):
            self.Limbnum = 7
        else:
            self.Limbnum = 0
        
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
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

        self.Speeds = Speeds

        self.Activated = 1 # Change this to zero if you don't want it to move

        self.HomePosition = Positions[0]

        self.FirstMovePosition = 0

        self.OffsetPercent = 0

        self.Phase = 0 # 2 for Stance, 1 for transition to stance, -2 for swing, -1 for transition to swing, 0 for other

        self.PresentPosition = 0

        self.GivenPosition = 0

        self.GivenSpeed = 0

        self.MovementTime = 0 # Length of time for the last movement given

        self.PhaseIndex = 0 

        self.StrideIndex = 0

        self.PhaseTime = 0

        self.StrideTime = 0

        self.TotalTime = 0


    def InitialSetup(self): # Use SetServoTraits to fill this
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

    def ToggleTorque(self,OnOrOff):
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

    def SetServoVelocity(self):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_PROFILE_VELOCITY, VELOCITY_LIMIT_H)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Velocity limit set to: %03d" %(self.ID, VELOCITY_LIMIT_H))

    def MoveServo(self,DesPos):
         # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, ADDR_PRO_GOAL_POSITION, DesPos)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Goal Position set to: %03d" %(self.ID, DesPos))

    def MoveHome(self):
        self.MoveServo(self.Positions[0][self.ID])


    def ContinuousMove(self):
        pass

    def StartTimer(self):
        pass

    def EndTimer(self):
        pass

    def DisplayTraitValues(self):
        pass

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
        sleep(2)

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
        self.ServoList = servolist
        getNum = lambda a : a.ID
        self.IDList = map(getNum,self.ServoList)
        listLen = len(self.IDList)
        rangeList = list(range(0,listLen))
        self.ServoDict = {}
        for i in rangeList:
            self.ServoDict[i] = self.IDList[i] 

        self.StrideCount = 1

        self.IsHome = None # Check all Servo Members to set this value

        self.PhaseTime = 0 # Maybe Not Needed? Each Servo also has this trait

        self.StrideTime = 0

        self.TotalTime = 0

        self.GoalVelocity = []
        self.GoalPosition = []

    def MoveLimb(self,IndexIn,portHandler,packetHandler):
         # Initialize GroupSyncWrite instance
        groupSyncWritePOS = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize GroupSyncWrite instance
        groupSyncWriteVEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_VELOCITY_LIMIT)

        # Initialize GroupSyncRead instace for Present Position
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

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

            dxl_addparam_result = groupSyncRead.addParam(d)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % d)
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

        PresentPositions = []

        for e in self.ServoList:
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

    def MoveHome(self,portHandler,packetHandler):
        self.MoveLimb(0,portHandler,packetHandler)
        print(f"Limb #{self.LimbNumber} Moved To Home Position\n")

    def ReadData(self,GroupSyncRead):
        pass

    def ContinuousMove(self,portHandler,packetHandler):
        global stopVal
        # Initialize GroupSyncRead instace for Moving Status
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_MOVING, LEN_MOVING)

        for h in self.ServoList:
            dxl_addparam_result = groupSyncRead.addParam(h)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % h)
                quit()
        
        index = 1
        isStopped = [0] * len(self.ServoList)
        timeMarkers = [0] * len(self.ServoList)
        while 1:
            self.MoveLimb(index,portHandler,packetHandler)
            index += 1
            if (stopVal == 1):
                break
            if (index > 21):
                index = 0
                self.StrideCount += 1

            while 1:
                # Syncread Moving Value
                dxl_comm_result = groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

                for i in self.ServoDict:
                    # Get Dynamixel#1 present Moving value
                    dxl_mov = groupSyncRead.getData(i, ADDR_MOVING, LEN_MOVING)
                    if (dxl_mov == 0) and (isStopped[i] == 0):
                        timeMarkers[i] = time.perf_counter()
                        isStopped[i] = 1
                        

    def UpdateValues(self):
        pass



class Leg(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

        self.Phase = [] # Check all Servo Members to set this value


class Neck(Limb):
    pass

class Spine(Limb):
    pass

class Tail(Limb):
    pass



class Body:
    def __init__(self,name):
        self.Name = name

    def MoveBody(self):
        pass

    def MoveHome(self):
        pass


class DataDocument:
    def __init__(self,directory,name,extension):
        self.Directory = directory
        self.Name = name
        self.FileExtension = extension

    def CreateDoc(self):
        pass

    def CheckForDoc(self):
        pass

    def WriteToDoc(self):
        pass

    def CloseDoc(self):
        pass

    def __del__(self):
        pass   

