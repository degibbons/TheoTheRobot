##### ALWAYS REMEMBER to run  ######
#    setup.py FIRST when moving 
#           to new computer

# NEED TO WRITE SCRIPT TO IMPORT LIBRARYS BEFORE RUNNING MASTER SCRIPT


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
        desired_movement = int(input("If you'd like to move servo #%d, is it to one position[1] or numerous positions[2]?: " % desired_servo_single))
        if (desired_movement == 1):
            desired_destination = int(input("Enter the destination you'd like to move servo #%d to: " % desired_servo_single))
            desired_speed = int(input("At what speed would you like servo #%d to move at?: " % desired_servo_single))
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0)
            SetSingleServoVelocity(desired_servo_single,desired_speed)
            MoveSingleServo(desired_servo_single,desired_destination)
        elif (desired_movement == 2):
            timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
            [speeds] = DetermineSpeeds(timespan,PositionsFile)
            [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
            SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0)
            index = 0
            while 1:
                SetSingleServoVelocity(desired_servo_single,SpeedsArray[desired_servo_single-1,index])
                MoveSingleServo(desired_servo_single,PositionsArray[index,desired_servo_single])
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
        [speeds] = DetermineSpeeds(timespan,PositionsFile)
        [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
        SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
        index = 1
        if (desired_servo_limb == 1):
            TurnOffOnTorque(TORQUE_ON,1,1,4)
            MoveSingleLimb(1,PositionsArray,SpeedsArray,index)
        elif (desired_servo_limb == 2):
            TurnOffOnTorque(TORQUE_ON,1,5,8)
            MoveSingleLimb(2,PositionsArray,SpeedsArray,index)
        elif (desired_servo_limb == 3):
            TurnOffOnTorque(TORQUE_ON,1,9,12)
            MoveSingleLimb(3,PositionsArray,SpeedsArray,index)
        elif (desired_servo_limb == 4):
            TurnOffOnTorque(TORQUE_ON,1,13,16)
            MoveSingleLimb(4,PositionsArray,SpeedsArray,index)
        elif (desired_servo_limb == 5):
            TurnOffOnTorque(TORQUE_ON,1,17,18)
            # Need Different Speed and Position Arrays for Neck
            pass
        elif (desired_servo_limb == 6):
            TurnOffOnTorque(TORQUE_ON,1,19,22)
            # Need Different Speed and Position Arrays for Spine
            pass
        elif (desired_servo_limb == 7):
            TurnOffOnTorque(TORQUE_ON,1,23,24)
            # Need Different Speed and Position Arrays for Tail
            pass
        else:
            pass
    elif (desired_action1 == 3):
        desired_action3 = int(input("Would you like to move the robot to Home Position[1]? Or through an entire stride[2]?"))
        if (desired_action3 == 1):
            StraightenSpine()
            MoveLimbsHome(PositionsArray,STRAIGHT_SPEED_ARRAY) #May not be ok as far as passing in just a list (need array?)
        elif (desired_action3 == 2):
            timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
            [speeds] = DetermineSpeeds(timespan,PositionsFile)
            [FrontSpeeds, BackSpeeds] = PostProcessSpeeds(speeds)
            SpeedsArray = np.concatenate((FrontSpeeds,BackSpeeds),axis=1)
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0)
            MoveEntireBody(PositionsArray,SpeedsArray)
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
            TurnOffOnTorque(TORQUE_OFF,1,1,24)
        elif (desired_action2 == 2):
            TurnOffOnTorque(TORQUE_ON,1,1,24)
        elif (desired_action2 == 3):
            desired_servo_single = int(input("Input the desired servo # you wish to alter: "))
            TurnOffOnTorque(TORQUE_OFF,0,desired_servo_single,0)
        elif (desired_action2 == 4):
            desired_servo_single = int(input("Input the desired servo # you wish to alter: "))
            TurnOffOnTorque(TORQUE_ON,0,desired_servo_single,0)
        elif (desired_action2 == 5):
            # Use ReturnRelevantData function
            pass
        elif (desired_action2 == 6):
            # Use ChangeSpecificTrait function
            pass
        else:
            pass
    elif (desired_action1 == 5):
        CleanUp(number_of_servos_connected)
        break
    else:
        pass
    