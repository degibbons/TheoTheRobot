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

# Break detection and assembly into its own function(s)

# Also make function for determining speeds from given float input

CurrentDoc = None

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
TheoBodyList = []
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
        TheoBodyList.append(FR_Leg)
        TheoBodyDict[1] = FR_Leg
        print("Front Right Limb is Digitally Assembled.")
    if (FL_limbCount == 4):
        FL_Limb = [ServoObjDict[5],ServoObjDict[6],ServoObjDict[7],ServoObjDict[8]]
        FL_Leg = Leg(2,FL_Limb)
        TheoBodyList.append(FL_Leg)
        TheoBodyDict[2] = FL_Leg
        print("Front Left Limb is Digitally Assembled.")
    if (BR_limbCount == 4):
        BR_Limb = [ServoObjDict[9],ServoObjDict[10],ServoObjDict[11],ServoObjDict[12]]
        BR_Leg = Leg(3,BR_Limb)
        TheoBodyList.append(BR_Leg)
        TheoBodyDict[3] = BR_Leg
        print("Back Right Limb is Digitally Assembled.")
    if (BL_limbCount == 4):
        BL_Limb = [ServoObjDict[13],ServoObjDict[14],ServoObjDict[15],ServoObjDict[16]]
        BL_Leg = Leg(4,BL_Limb)
        TheoBodyList.append(BL_Leg)
        TheoBodyDict[4] = BL_Leg
        print("Back Left Limb is Digitally Assembled.")
    if (Neck_limbCount == 2):
        Neck_Limb = [ServoObjDict[17],ServoObjDict[18]]
        NeckStructure = Neck(5,Neck_Limb)
        TheoBodyList.append(NeckStructure)
        TheoBodyDict[5] = NeckStructure
        print("Neck Limb is Digitally Assembled.")
    if (Spine_limbCount == 4):
        Spine_Limb = [ServoObjDict[19],ServoObjDict[20],ServoObjDict[21],ServoObjDict[22]]
        SpineStructure = Spine(6,Spine_Limb)
        TheoBodyList.append(SpineStructure)
        TheoBodyDict[6] = SpineStructure
        print("Spine Limb is Digitally Assembled.")
    if (Tail_limbCount == 2):
        Tail_Limb = [ServoObjDict[23],ServoObjDict[24]]
        TailStructure = Tail(7,Tail_Limb)
        TheoBodyList.append(TailStructure)
        TheoBodyDict[7] = TailStructure
        print("Tail Limb is Digitally Assembled.")

TheoBody = Body(TheoBodyList)
# NEED LIST OF LIMBS TO INITIATE BODY STRUCTURE

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
            desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
            speeds = DetermineSpeeds(desired_timespan,PositionsFile)
            TotMatrix_speeds = PostProcessSpeeds(speeds)
            ServoObjDict[desired_servo].Speeds = TotMatrix_speeds[:][desired_servo]
            ServoObjDict[desired_servo].ContinuousMove[portHandler,packetHandler]
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
        desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
        speeds = DetermineSpeeds(desired_timespan,PositionsFile)
        TotMatrix_speeds = PostProcessSpeeds(speeds)
        TheoBodyDict[desired_servo_limb].MoveHome(portHandler,packetHandler,ReadOption=False)
        for each_servo in TheoBodyDict[desired_servo_limb].ServoList:
            each_servo.Speeds = TotMatrix_speeds[:][each_servo.ID]
        if (desired_movement.lower() == 'o'):
            desired_position = int(input("To what location index do you want the limb to move?: "))
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
            while 1:
                if getch() == chr(0x0D):
                    break
            TheoBodyDict[desired_servo_limb].MoveLimb(desired_position,portHandler,packetHandler,ReadOption=False)
        elif (desired_movement.lower() == 'c'):
            print("\n")
            print("Press Enter to start when ready.")
            print("When done, hit Escape.\n")
            while 1:
                if getch() == chr(0x0D):
                    break
            TheoBodyDict[desired_servo_limb].ContinuousMove(portHandler,packetHandler)
        
    elif(desired_action == 3): # Move entire robot
        print("All available Servos will run their given movement instructions.")
        desired_timespan = float(input("How long (in seconds) do you want a single stride to take?: "))
        speeds = DetermineSpeeds(desired_timespan,PositionsFile)
        TotMatrix_speeds = PostProcessSpeeds(speeds)
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
        TheoBody.ContinuousMove(portHandler,packetHandler)
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
        CleanUp()
        ShutDown()

if __name__ == "__main__":
    # Run Main Script
else:
    # Run Setup? Or place in separate code?