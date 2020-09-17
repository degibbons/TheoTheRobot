##### ALWAYS REMEMBER to run  ######
#    setup.py FIRST when moving 
#           to new computer

# NEED TO WRITE SCRIPT TO IMPORT LIBRARYS BEFORE RUNNING MASTER SCRIPT
# Need pandas, numpy, and dynamixel_sdk

### NEED ERROR CHECKING PORTIONS

# Need to implement threading to stop movement when button is pressed during cycling

# Need continuous loop asking user for options instead of just once through

import os
import numpy as np
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
            MoveSingleLimb(1,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
        elif (desired_servo_limb == 2):
            limb = F_L_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,5,8,portHandler,packetHandler)
            MoveSingleLimb(2,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
        elif (desired_servo_limb == 3):
            limb = B_R_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,9,12,portHandler,packetHandler)
            MoveSingleLimb(3,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
        elif (desired_servo_limb == 4):
            limb = B_L_ARM
            for i in range(limb[0],limb[3]+1):
                SetServoTraits(i,portHandler,packetHandler)
            TurnOffOnTorque(TORQUE_ON,1,13,16,portHandler,packetHandler)
            MoveSingleLimb(4,PositionsArray,SpeedsArray,index,portHandler,packetHandler)
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
            # User stop built into function, needs to be extracted and put into MasterScript
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
        print("6: Send Servo trait\n")
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
            # Use ReturnRelevantData function
            pass
        elif (desired_action2 == 6):
            # Use ChangeSpecificTrait function
            pass
        else:
            pass
    elif (desired_action1 == 5):
        CleanUp(number_of_servos_connected,portHandler,packetHandler)
        break
    else:
        pass
    