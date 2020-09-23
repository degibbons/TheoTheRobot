##### ALWAYS REMEMBER to run  ######
#    setup.py FIRST when moving 
#           to new computer

# GUI to make interaction easier (consider tkinter)

# NEED TO WRITE SCRIPT TO IMPORT LIBRARYS BEFORE RUNNING MASTER SCRIPT
# Need pandas, numpy, and dynamixel_sdk

### NEED ERROR CHECKING PORTIONS

# Need to implement threading to stop movement when button is pressed during cycling

import os
import numpy as np
import time
from ControlTable import *
from RelevantFunctions import *
from dynamixel_sdk import *

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

number_of_servos_connected = int(input("Enter the number of servos: "))

while 1:
    PrintUserMenu()

    desired_action1 = int(input("Enter selection number here: "))

    if (desired_action1 == 1):
        desired_servo_single = int(input("Input the desired servo # you wish to move: "))
        SetServoTraits(desired_servo_single,portHandler,packetHandler)
        desired_movement = int(input("If you'd like to move servo #%d, is it to one position[1] or numerous positions[2]?: " % desired_servo_single))
        if (desired_movement == 1):
            desired_destination = int(input("Enter the destination you'd like to move servo #%d to: " % desired_servo_single))
            desired_speed = int(input("At what speed would you like servo #%d to move at?: " % desired_servo_single))
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0,portHandler,packetHandler)
            SetSingleServoVelocity(desired_servo_single,desired_speed,portHandler,packetHandler)
            MoveSingleServo(desired_servo_single,desired_destination,portHandler,packetHandler)
        elif (desired_movement == 2):
            timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
            speeds = DetermineSpeeds(timespan,PositionsFile)
            [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
            SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0,portHandler,packetHandler)
            index = 0
            while 1:
                SetSingleServoVelocity(desired_servo_single,SpeedsArray[index,desired_servo_single-1],portHandler,packetHandler)
                MoveSingleServo(desired_servo_single,PositionsArray[index,desired_servo_single],portHandler,packetHandler)
                index += 1
                if (index > 21):
                    index = 0
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
        else:
            pass
    elif (desired_action1 == 2):
        print("\nThe limbs are laid out as follows:")
        print("1: Front Right Limb")
        print("2: Front Left Limb")
        print("3: Back Right Limb")
        print("4: Back Left Limb")
        print("5: Spine")
        print("6: Neck")
        print("7: Tail\n")
        desired_servo_limb = int(input("Enter selection number here: "))
        timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
        speeds = DetermineSpeeds(timespan,PositionsFile)
        [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
        SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
        index = 1
        if (desired_servo_limb == 1):
            limb = F_R_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,1,4,portHandler,packetHandler)
            while 1:
                MoveSingleLimb(1,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
                index += 1
                if (index > 21):
                    index = 0
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
        elif (desired_servo_limb == 2):
            limb = F_L_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,5,8,portHandler,packetHandler)
            while 1:
                MoveSingleLimb(2,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
                index += 1
                if (index > 21):
                    index = 0
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
        elif (desired_servo_limb == 3):
            limb = B_R_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,9,12,portHandler,packetHandler)
            while 1:
                MoveSingleLimb(3,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
                index += 1
                if (index > 21):
                    index = 0
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
        elif (desired_servo_limb == 4):
            limb = B_L_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,13,16,portHandler,packetHandler)
            while 1:
                MoveSingleLimb(4,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
                index += 1
                if (index > 21):
                    index = 0
                print("Press any key to continue! (or press ESC to quit!)")
                if getch() == chr(0x1b):
                    break
        elif (desired_servo_limb == 5):
            limb = NECK
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,17,18,portHandler,packetHandler)
            # Need Different Speed and Position Arrays for Neck
            pass
        elif (desired_servo_limb == 6):
            limb = SPINE
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,19,22,portHandler,packetHandler)
            # Need Different Speed and Position Arrays for Spine
            pass
        elif (desired_servo_limb == 7):
            limb = TAIL
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,23,24,portHandler,packetHandler)
            # Need Different Speed and Position Arrays for Tail
            pass
        else:
            pass
    elif (desired_action1 == 3):
        for i in range(limb[0],limb[24]+1):
            SetServoTraits(i,portHandler,packetHandler)
        desired_action3 = int(input("Would you like to move the robot to Home Position[1]? Or through an entire stride[2]?"))
        if (desired_action3 == 1):
            StraightenSpine(portHandler,packetHandler)
            MoveLimbsHome(PositionsArray,STRAIGHT_SPEED_ARRAY,portHandler,packetHandler) #May not be ok as far as passing in just a list (need array?)
        elif (desired_action3 == 2):
            timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
            speeds = DetermineSpeeds(timespan,PositionsFile)
            [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
            SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0,portHandler,packetHandler)
            MoveEntireBody(PositionsArray,SpeedsArray,portHandler,packetHandler)
            # User stop needs to be added
            # Threading should be used here
        else:
            pass
    elif (desired_action1 == 4):
        print("---------SUBMENU---------")
        print("1: Turn Torque setting off for all servos")
        print("2: Turn Torque setting on for all servos")
        print("3: Turn Torque setting off for specific servo")
        print("4: Turn Torque setting on for specific servo")
        print("5: Get Servo trait")
        print("6: Send Servo trait")
        print("7: Ping all connected Servos")
        print("8: Reboot specified Servo(s)")
        print("9: Reset specified Servo(s)\n")
        desired_action2 = int(input("Enter selection number here: "))
        if (desired_action2 == 1):
            TurnOffOnTorque(TORQUE_OFF,1,1,24,portHandler,packetHandler)
        elif (desired_action2 == 2):
            TurnOffOnTorque(TORQUE_ON,1,1,24,portHandler,packetHandler)
        elif (desired_action2 == 3):
            desired_servo_single = int(input("Input the desired servo # you wish to alter: "))
            TurnOffOnTorque(TORQUE_OFF,0,desired_servo_single,0,portHandler,packetHandler)
        elif (desired_action2 == 4):
            desired_servo_single = int(input("Input the desired servo # you wish to alter: "))
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0,portHandler,packetHandler)
        elif (desired_action2 == 5):
            DisplayServoTraits()
            Selection1 = int(input("Enter the Trait Number you'd like to extract: "))
            Selection2 = int(input("Enter the Servo you'd like to extract the Trait from: "))
            ReadTraitData(Selection1,Selection2)
            print("Trait Read Finished\n")
        elif (desired_action2 == 6):
            DisplayServoTraits()
            Selection1 = int(input("Enter the Trait Number you'd like to edit: "))
            Selection2 = int(input("Enter the Servo you'd like to edit the Trait for: "))
            Selection3 = int(input("Enter the Value you'd like to change the trait to: "))
            WriteTraitData(Selection1,Selection3,Selection2)
            print("Trait Edit Finished\n")
        elif (desired_action2 == 7):
            PingServos()
        elif (desired_action2 == 8):
            AllOrOne = input("Are you Rebooting All[2] or One[1] of the servos?: ")
            if (AllOrOne == 1):
                DesiredServo = input("What Servo do you want to reboot?: ")
                RebootServos(DesiredServo)
            elif (AllOrOne == 2):
                for i in range(1,25):
                    RebootServos(i)
                    print("Dynamixel #%d is now Rebooted." % (DesiredServo))
            else:
                pass
        elif (desired_action2 == 9):        
            AllOrOne = input("Are you Resetting All[2] or One[1] of the servos?: ")
            if (AllOrOne == 1):
                DesiredServo = input("What Servo do you want to reset?: ")
                ResetServos(DesiredServo)
            elif (AllOrOne == 2):
                for i in range(1,25):
                    ResetServos(i)
                    print("Dynamixel #%d is now Reset." % (DesiredServo))
            else:
                pass
        else:
            pass
    elif (desired_action1 == 5):
        CleanUp(number_of_servos_connected,portHandler,packetHandler)
        break
    else:
        pass
    