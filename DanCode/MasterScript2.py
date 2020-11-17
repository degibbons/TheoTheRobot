# Ping Servos to See Which Ones Are Connected

# Assign Servo Object To Each One

# Assemble Limb Objects Using Present Servos


import os
import numpy as np
import time
from ClassDefinitions import *
from ControlTable import *
from RelevantFunctions import *
from dynamixel_sdk import *
from threading import Thread


[portHandler, packetHandler] = InitialSetup()              # Uses Dynamixel SDK library

[FrontPositions, BackPositions] = PostProcessPositions()
PositionsArray = np.concatenate((FrontPositions,BackPositions),axis=1)

ServoObjList = []
ServoObjDict = {}
dxl_data_list = PingServos()
for dxl_id in dxl_data_list:
    print("[ID:%03d] Detected" % (dxl_id))
    RelativeServo = Servo(dxl_id,PositionsArray[:][dxl_id])
    ServoObjList.append(RelativeServo)
    ServoObjDict[dxl_id] = RelativeServo

NumberOfServosConnected = len(dxl_data_list)

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
TheoBody = []
TheoBodyDict = {}
for ServoID,ServoObj in ServoObjDict:
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
        TheoBody.append(FR_Leg)
        TheoBodyDict[1] = FR_Leg
        print("Front Right Limb is Digitally Assembled.")
    if (FL_limbCount == 4):
        FL_Limb = [ServoObjDict[5],ServoObjDict[6],ServoObjDict[7],ServoObjDict[8]]
        FL_Leg = Leg(2,FL_Limb)
        TheoBody.append(FL_Leg)
        TheoBodyDict[2] = FL_Leg
        print("Front Left Limb is Digitally Assembled.")
    if (BR_limbCount == 4):
        BR_Limb = [ServoObjDict[9],ServoObjDict[10],ServoObjDict[11],ServoObjDict[12]]
        BR_Leg = Leg(3,BR_Limb)
        TheoBody.append(BR_Leg)
        TheoBodyDict[3] = BR_Leg
        print("Back Right Limb is Digitally Assembled.")
    if (BL_limbCount == 4):
        BL_Limb = [ServoObjDict[13],ServoObjDict[14],ServoObjDict[15],ServoObjDict[16]]
        BL_Leg = Leg(4,BL_Limb)
        TheoBody.append(BL_Leg)
        TheoBodyDict[4] = BL_Leg
        print("Back Left Limb is Digitally Assembled.")
    if (Neck_limbCount == 2):
        Neck_Limb = [ServoObjDict[17],ServoObjDict[18]]
        NeckStructure = Neck(5,Neck_Limb)
        TheoBody.append(NeckStructure)
        TheoBodyDict[5] = NeckStructure
        print("Neck Limb is Digitally Assembled.")
    if (Spine_limbCount == 4):
        Spine_Limb = [ServoObjDict[19],ServoObjDict[20],ServoObjDict[21],ServoObjDict[22]]
        SpineStructure = Spine(6,Spine_Limb)
        TheoBody.append(SpineStructure)
        TheoBodyDict[6] = SpineStructure
        print("Spine Limb is Digitally Assembled.")
    if (Tail_limbCount == 2):
        Tail_Limb = [ServoObjDict[23],ServoObjDict[24]]
        TailStructure = Tail(7,Tail_Limb)
        TheoBody.append(TailStructure)
        TheoBodyDict[7] = TailStructure
        print("Tail Limb is Digitally Assembled.")

while 1:
    PrintUserMenu()
    print("\n")
    desired_action_1 = int(input("Enter selection number here: "))
    print("\n\n")
    if (desired_action_1 == 1): # Move 1 Servo
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
            desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
            speeds = DetermineSpeeds(desired_timespan,PositionsFile)
            TotMatrix_speeds = PostProcessSpeeds(speeds)
            ServoObjDict[desired_servo].Speeds = TotMatrix_speeds[:][desired_servo]
            ServoObjDict[desired_servo].ContinuousMove[portHandler,packetHandler]
    elif(desired_action_1 == 2): # Move 1 Limb
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
        if (desired_movement.lower() == 'o'):
            desired_position = int(input("To what location index do you want the limb to move?: "))
            print("\n")
            desired_timespan = float(input("What speed do you want the limb to move at?: "))
            print("\n")
        # NEEDS TO BE FILLED IN HERE
        elif (desired_movement.lower() == 'c'):
            desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
            speeds = DetermineSpeeds(desired_timespan,PositionsFile)
            TotMatrix_speeds = PostProcessSpeeds(speeds)
        # NEEDS TO BE FILLED IN HERE USING STUFF BELOW
        speeds = DetermineSpeeds(desired_timespan,PositionsFile)
        TotMatrix_speeds = PostProcessSpeeds(speeds)
        for each_servo in TheoBodyDict[desired_servo_limb].ServoList:
            each_servo.Speeds = TotMatrix_speeds[:][each_servo.ID]
        TheoBodyDict[desired_servo_limb].
        
    elif(desired_action_1 == 3): # Move entire robot
        pass
    elif(desired_action_1 == 4): # Perform a specific non-movement action
        pass
    elif(desired_action_1 == 5): # Shut down robot, delete object structures, and close documents
        pass


