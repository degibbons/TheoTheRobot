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

[portHandler, packetHandler] = InitialSetup()              # Uses Dynamixel SDK library


(FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4,
 HL_TOT_R_1, HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, HL_TOT_L_3, HL_TOT_L_4) = ReadServoAngles(PositionsFile)

PositionsMatrix = PostProcessPositions(FL_TOT_R_1, FL_TOT_R_2, FL_TOT_R_3, FL_TOT_R_4, FL_TOT_L_1, FL_TOT_L_2, FL_TOT_L_3, FL_TOT_L_4,
 HL_TOT_R_1, HL_TOT_R_2, HL_TOT_R_3, HL_TOT_R_4, HL_TOT_L_1, HL_TOT_L_2, HL_TOT_L_3, HL_TOT_L_4)

[ServoObjList, ServoObjDict, TheoLimbList, TheoLimbDict, TheoBody] = AssembleRobot(PositionsMatrix)
desired_servo_limb = 1
while 1:
    desired_action = int(input("Run Test Protocol?(1) or Shutdown Sequence?(2):"))
    if (desired_action == 1):
        print("Running Auto Continuous Move Protocol...\n")
        matrix_speeds = SpeedMerge()
        for each_servo in TheoLimbDict[desired_servo_limb].ServoList:
            each_servo.InitialSetup()
            each_servo.ToggleTorque(1,portHandler,packetHandler)
            each_servo.Speeds = matrix_speeds[:,each_servo.ID-1]
        print("\nMoving Limb #{s_l} Home:\n".format(s_l=TheoLimbDict[desired_servo_limb]))
        TheoLimbDict[desired_servo_limb].MoveHome(portHandler,packetHandler)
        print("\nFinished Moving\n")
        print("Press Enter to start when ready.")
        print("When done, hit Escape.\n")
        while 1:
            if getch() == chr(0x0D):
                break
        RunThreads(TheoLimbDict[desired_servo_limb],portHandler,packetHandler)
    elif (desired_action == 2): # Shut down robot, delete object structures, and close documents
        CleanUp(TheoBody,TheoLimbList, ServoObjList,CurrentDoc)
        ShutDown()
        break
    else:
        print("That's not a recognized option, please try again.\n")
        continue

# if __name__ == "__main__":
#     # Run Main Script
#     pass
# else:
#     # Run Setup? Or place in separate code?
#     pass