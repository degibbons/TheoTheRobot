### Theo Master Script To Be Run After Pre-Installation Script is Run 
# (Pre-Installation Script will install all libraries needed to run Master Script)
# (Will also create a Data subdirectory and prepackage processing code for post-data analysis)
# Author: Dan Gibbons
# Version: 0.0.1
###

import os
import numpy as np
import time
from ClassDefinitions import *
from ControlTable import *
from RelevantFunctions import *
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

[FrontPositions, BackPositions] = PostProcessPositions()
PositionsArray = np.concatenate((FrontPositions,BackPositions),axis=1)

CurrentDoc = None

[ServoObjList, ServoObjDict, TheoLimbList, TheoLimbDict, TheoBody] = AssembleRobot(PositionsArray)

while 1:
    PrintUserMenu()
    print("\n")
    desired_action = int(input("Enter selection number here: "))
    print("\n\n")
    if (desired_action == 1): # Move 1 Servo
        desired_servo = int(input("Please enter the desired servo # you wish to move: "))
        print("\n")
        ServoObjDict[desired_servo].InitialSetup
        ServoObjDict[desired_servo].ToggleTorque(1,portHandler,packetHandler)
        desired_origin = input("Would you like this servo to move to it's natural Home Position?[Y/n]: ")
        print("\n")
        if (desired_origin.lower()=='y'):
            ServoObjDict[desired_servo].MoveHome()
        else:
            pass
        desired_movement = input("Would you like the servo to move One Time or Continuously?[o/c]: ")
        print("\n")
        if (desired_movement.lower() == 'o'):
            desired_position = int(input("To what location do you want the servo to move?: "))
            print("\n")
            desired_speed = int(input("At what speed do you wan't the servo to move at?: "))
            print("\n")
            ServoObjDict[desired_servo].SetServoVelocity(desired_speed)
            ServoObjDict[desired_servo].MoveServo(desired_position)
        elif (desired_movement.lower() == 'c'):
            TotMatrix_speeds = SpeedMerge()
            ServoObjDict[desired_servo].Speeds = TotMatrix_speeds[:][desired_servo]
            RunThreads(ServoObjDict[desired_servo],portHandler,packetHandler)
    elif(desired_action == 2): # Move 1 Limb
        print("\nThe limbs are laid out as follows:")
        print("1: Front Right Limb")
        print("2: Front Left Limb")
        print("3: Back Right Limb")
        print("4: Back Left Limb")
        print("5: Spine")
        print("6: Neck")
        print("7: Tail\n")
        desired_servo_limb = int(input("Enter selection number here: "))
        print("\n")
        desired_movement = input("Would you like the limb to move One Time or Continuously?[o/c]: ")
        TotMatrix_speeds = SpeedMerge()
        TheoLimbDict[desired_servo_limb].MoveHome(portHandler,packetHandler,ReadOption=False)
        for each_servo in TheoLimbDict[desired_servo_limb].ServoList:
            each_servo.Speeds = TotMatrix_speeds[:][each_servo.ID]
        if (desired_movement.lower() == 'o'):
            desired_position = int(input("To what location index do you want the limb to move?: "))
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
            while 1:
                if getch() == chr(0x0D):
                    break
            TheoLimbDict[desired_servo_limb].MoveLimb(desired_position,portHandler,packetHandler,ReadOption=False)
        elif (desired_movement.lower() == 'c'):
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
            while 1:
                if getch() == chr(0x0D):
                    break
            RunThreads(TheoLimbDict[desired_servo_limb],portHandler,packetHandler)
        
    elif(desired_action == 3): # Move entire robot
        print("All available Servos will run their given movement instructions.")
        TotMatrix_Speeds = SpeedMerge()
        TheoBody.MoveHome(portHandler,packetHandler)
        desired_movement = input("Would you like the limb to move One Time or Continuously?[o/c]: ")
        if (desired_movement.lower() == 'o'):
            desired_position = int(input("To what location index do you want the body to move?: "))
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
            while 1:
                if getch() == chr(0x0D):
                    break
            TheoBody.MoveBody(desired_position,portHandler,packetHandler)
        elif (desired_movement.lower() == 'c'):
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
        while 1:
            if getch() == chr(0x0D):
                break
        RunThreads(TheoBody,portHandler,packetHandler)
    elif(desired_action == 4): # Perform a specific non-movement action
        PrintOptionsSubmenu()
        desired_selection = int(input("Enter selection number here: "))
        print("\n")
        if (desired_selection == 1):
            for each_servo in ServoObjList:
                each_servo.ToggleTorque(0,portHandler,packetHandler)
        elif (desired_selection == 2):
            for each_servo in ServoObjList:
                each_servo.ToggleTorque(1,portHandler,packetHandler)
        elif (desired_selection == 3):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            ServoObjDict[desired_servo].ToggleTorque(0,portHandler,packetHandler)
        elif (desired_selection == 4):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            ServoObjDict[desired_servo].ToggleTorque(1,portHandler,packetHandler)
        elif (desired_selection == 5):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            DisplayServoTraits()
            desired_trait = int(input("Enter the Trait Number you would like to extract: "))
            ServoObjDict[desired_servo].ReadTrait(desired_trait)
        elif (desired_selection == 6):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            DisplayServoTraits()
            desired_trait = int(input("Enter the Trait Number you would like to edit: "))
            desired_value = int(input("Enter the value to write to the previous Trait Address: "))
            ServoObjDict[desired_servo].WriteTrait(desired_trait,desired_value)
        elif (desired_selection == 7):
            PingServos()
        elif (desired_selection == 8):
            for each_servo in ServoObjList:
                each_servo.RebootServo()
        elif (desired_selection == 9):
            for each_servo in ServoObjList:
                each_servo.ResetServo()
        elif (desired_selection == 10):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            ServoObjDict[desired_servo].RebootServo()
        elif (desired_selection == 11):
            desired_servo = int(input("Input the desired servo # you wish to apply your action to: "))
            print("\n")
            ServoObjDict[desired_servo].ResetServo()
        elif (desired_selection == 12):
            # Need peripheral code here for GPIO pin commands
            pass
    elif(desired_action == 5): # Shut down robot, delete object structures, and close documents
        CleanUp(TheoBody,TheoLimbList, ServoObjList,CurrentDoc)
        ShutDown()

if __name__ == "__main__":
    # Run Main Script
    pass
else:
    # Run Setup? Or place in separate code?
    pass