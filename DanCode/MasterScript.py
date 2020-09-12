##### ALWAYS REMEMBER to run  ######
#    setup.py FIRST when moving 
#           to new computer


### NEED ERROR CHECKING PORTIONS

import os
import ControlTable
import RelevantFunctions
from dynamixel_sdk import *     

InitialSetup()               # Uses Dynamixel SDK library

number_of_servos_connected = int(input("Enter the number of servos: "))

print("\nWhat would you like to do?\n")
print("---------MENU---------")
print("1: Move a single servo")
print("2: Move a specific limb")
print("3: Move the entire robot")
print("4: Other\n")

desired_action1 = int(input("Enter selection number here: "))

if (desired_action1 == 1):
    desired_servo_single = int(input("Input the desired servo # you wish to move: "))
    desired_movement = int(input("If you'd like to move servo #%d, is it to one position[1] or numerous positions[2]?: " % desired_servo_single))
    if (desired_movement == 1):
        desired_destination = int(input("Enter the destination you'd like to move servo #%d to: " % desired_servo_single))
        desried_speed = int(input("At what speed would you like servo #%d to move at?: " % desired_servo_single"))
    elif (desired_movement == 2):
        desired_destination_array = int(input("Enter the destination array you'd like to move servo #%d to: " % desired_servo_single))
        desired_speed_array = int(input("Please enter the speed array you'd like to move servo #%d at: " % desired_servo_single))
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
    if (desired_servo_limb == 1):
        pass
    elif (desired_servo_limb == 2):
        pass
    elif (desired_servo_limb == 3):
        pass
    elif (desired_servo_limb == 4):
        pass
    elif (desired_servo_limb == 5):
        pass
    elif (desired_servo_limb == 6):
        pass
    elif (desired_servo_limb == 7):
        pass
    else:
        pass
elif (desired_action1 == 3):
    desired_action3 = int(input("Would you like to move the robot to Home Position[1]? Or through an entire stride[2]?"))
    if (desired_action3 == 1):
        pass
    elif (desired_action3 == 2):
        timespan = float(input("Please enter the amount of seconds you'd like one stride to take: "))
    else:
        pass
    

elif (desired_action1 == 4):
    print("---------SUBMENU---------")
    print("1: Turn Torque setting off for all servos")
    print("2: Turn Torque setting on for all servos")
    print("3: Turn Torque setting off for specific servo")
    print("4: Turn Torque setting on for specific servo")
    desired_action2 = int(input("Enter selection number here: "))
    if (desired_action2 == 1):
        pass
    elif (desired_action2 == 2):
        pass
    elif (desired_action2 == 3):
        pass
    elif (desired_action2 == 4):
        pass
    else:
        pass
else:
    pass