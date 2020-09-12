#ARBITRARY VALUES DESGINED FOR THEO
BAUDRATE                    = 1000000 #57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
PROTOCOL_VERSION            = 2
DRIVE_MODE_VEL_BASED = 0            #establish default drive mode, time or vel based
OPERATING_JOINT_POSITION_MODE = 3
DXL_ID = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
DXL1_ID = 1
DXL2_ID = 2
DXL3_ID = 3
DXL4_ID = 4
DXL5_ID = 5
DXL6_ID = 6
DXL7_ID = 7
DXL8_ID = 8
DXL9_ID = 9
DXL10_ID = 10
DXL11_ID = 11
DXL12_ID = 12
DXL13_ID = 13
DXL14_ID = 14
DXL15_ID = 15
DXL16_ID = 16
DXL17_ID = 17
DXL18_ID = 18
DXL19_ID = 19
DXL20_ID = 20
DXL21_ID = 21
DXL22_ID = 22
DXL23_ID = 23
DXL24_ID = 24

F_R_ARM_2_TEST = [DXL1_ID, DXL2_ID]
F_R_ARM = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID]
F_L_ARM = [DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID]
B_R_ARM = [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID]
B_L_ARM = [DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]
NECK    = [DXL17_ID, DXL18_ID]
SPINE   = [DXL19_ID, DXL20_ID, DXL21_ID, DXL22_ID]
TAIL    = [DXL23_ID, DXL24_ID]

#for homing movement
HOME_MOVING_STATUS_THRESHOLD = 2
THEO_GENERIC_HOME_POSITION=2048     #can be augmented for each limb/ servo etc
THEO_HOME_PROF_VEL  = 100

#default limits
VELOCITY_LIMIT_H = 1023
VELOCITY_LIMIT_L = 0
MAX_POSITION_LIMIT = 4095
MIN_POSITION_LIMIT = 0
ACCELERATION_LIMIT_H = 32767
ACCELERATION_LIMIT_L = 1
ACCELERATION_LIMIT_M = 1000
MOVING_THRESHOLD_ACCURACY_L = 20
MOVING_THRESHOLD_ACCURACY_M = 10
MOVING_THRESHOLD_ACCURACY_H = 1