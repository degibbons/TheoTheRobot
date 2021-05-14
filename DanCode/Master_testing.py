import os
import numpy as np
import time
from CF_testing import *
from ControlTable import *
import pandas as pd
#from RelevantFunctions import *
from dynamixel_sdk import *
from threading import Thread

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

[portHandler_1, portHandler_2, portHandler_3, portHandler_4, packetHandler] = InitialSetup()              # Uses Dynamixel SDK library
print("\nPort Handler's Have Been Initiated\n")

(FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4,
 HL_TOT_R_1, HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, HL_TOT_L_3, HL_TOT_L_4) = ReadServoAngles(PositionsFile)
print("Servo EXCEL data has been read.")

PositionsMatrix = PostProcessPositions(FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4,
 HL_TOT_R_1, HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, HL_TOT_L_3, HL_TOT_L_4)
print("Created Positions Matrix.")

[ServoObjList, ServoObjDict, TheoLimbList, TheoLimbDict, TheoBody] = AssembleRobot(PositionsMatrix,portHandler_1,portHandler_2,portHandler_3,packetHandler)
#desired_servo_limb = 1
while 1:
    desired_action = int(input("Run Test Protocol?(1), Shutdown Sequence?(2), or Reboot All(3):"))
    if (desired_action == 1):
        print("Running Auto Continuous Move Protocol...\n")
        matrix_speeds = SpeedMerge(PositionsMatrix)
        # Initialize Spine
        for each_limb in MainBodyLimbs:
            portHandlerX = portHandler_3
            for each_servo in TheoLimbDict[each_limb].ServoList:
                each_servo.InitialSetup(portHandlerX)
                each_servo.ToggleTorque(1,portHandlerX,packetHandler)
        # Initialize Legs
        for each_limb in LegLimbs:
            if (each_limb == 1 or each_limb == 2):
                portHandlerX = portHandler_1
            elif (each_limb == 3 or each_limb == 4):
                portHandlerX = portHandler_2
            for each_servo in TheoLimbDict[each_limb].ServoList:
                each_servo.InitialSetup(portHandlerX)
                each_servo.ToggleTorque(1,portHandlerX,packetHandler)
                each_servo.Speeds = matrix_speeds[:][each_servo.ID-1]
        TheoBody.MoveSpineHome(portHandler_3,packetHandler)
        TheoBody.MoveLegsHome(portHandler_1,portHandler_2,packetHandler)
        print("\nFinished Moving Home...\n")
        print("Press Enter to start when ready.")
        print("When done, hit Escape.\n")
        while 1:
            if getch() == chr(0x0D):
                break
        RunThreads(TheoBody,portHandler_1,portHandler_2,packetHandler)
    elif (desired_action == 2): # Shut down robot, delete object structures, and close documents
        CleanUp(TheoBody,TheoLimbList, ServoObjList,portHandler_1,portHandler_2,portHandler_3,packetHandler)
        ShutDown()
        break
    elif (desired_action == 3): # Reboot all servos
        for i in list(range(1,25)):
            if (i>=1 and i<=8):
                ServoObjDict[i].RebootServo(portHandler_1,packetHandler)
            elif (i>=9 and i<=16):
                ServoObjDict[i].RebootServo(portHandler_2,packetHandler)
            elif (i>= 17 and i<= 24):
                ServoObjDict[i].RebootServo(portHandler_3,packetHandler)
    else:
        print("That's not a recognized option, please try again.\n")
        continue

# if __name__ == "__main__":
#     # Run Main Script
#     pass
# else:
#     # Run Setup? Or place in separate code?
#     pass