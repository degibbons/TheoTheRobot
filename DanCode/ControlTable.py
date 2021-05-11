# Control table address and Values
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_VELOCITY_LIMIT         = 44            #4
ADDR_DRIVE_MODE             = 10            #1          0       # |0=vELOCITY BASED| 1=TIME_BASED|
DRIVE_MODE_VEL_BASED        = 0            #establish default drive mode, time or vel based
ADDR_OPERATING_MODE         = 11            #1          3       # |0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of spe ed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
OPERATING_JOINT_POSITION_MODE = 3
ADDR_ACCELERATION_LIMIT     = 40            #4          32767
ACCELERATION_LIMIT_M        = 1000
ADDR_MAX_POSITION_LIMIT     = 48            #4          4095
ADDR_MIN_POSITION_LIMIT     = 52            #4          0
MAX_POSITION_LIMIT          = 4095
MIN_POSITION_LIMIT          = 0
ADDR_MOVING_THRESHOLD       = 24            #4          10      #ESSENTIALLY ACCURACY TOLERANCE
MOVING_THRESHOLD_ACCURACY_H = 1
VELOCITY_LIMIT_H            = 1023
ADDR_PROFILE_VELOCITY       = 112
ADDR_MOVING                 = 122 

PreferedDelay               = 0.01

# Control table address
ADDR_PRO_BAUDRATE           = 8                             # Control table address is different in Dynamixel model

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Positions File Name
PositionsFile = "Kinematics_5.csv"

# Default setting
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

F_R_ARM = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID]
F_L_ARM = [DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID]
B_R_ARM = [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID]
B_L_ARM = [DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]
NECK    = [DXL17_ID, DXL18_ID]
SPINE   = [DXL19_ID, DXL20_ID, DXL21_ID, DXL22_ID]
TAIL    = [DXL23_ID, DXL24_ID]

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME_1                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
DEVICENAME_2                  = '/dev/ttyUSB1'
DEVICENAME_3                  = '/dev/ttyUSB2'
DEVICENAME_4                  = '/dev/ttyUSB3'                                            

FACTORYRST_DEFAULTBAUDRATE  = 57600                         # Dynamixel baudrate set by factoryreset
NEW_BAUDNUM                 = 3                             # New baudnum to recover Dynamixel baudrate as it was
OPERATION_MODE              = 0x02                          # 0xFF : reset all values
                                                            # 0x01 : reset all values except ID
                                                            # 0x02 : reset all values except ID and baudrate

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

TORQUE_ON                   = 1                 # Value for enabling the torque
TORQUE_OFF                  = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_VELOCITY_LIMIT          = 4
LEN_MOVING                  = 1

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

LegLimbs = [1, 2, 3, 4]
MainBodyLimbs = [5, 6, 7]
NeckLimb = 5
SpineLimb = 6
TailLimb = 7
DEVICENAMES = [DEVICENAME_1, DEVICENAME_2, DEVICENAME_3, DEVICENAME_4]

LimbNames = {1:"Right Front", 2:"Left Front", 3:"Right Back", 4:"Left Back", 5:"Neck", 6:"Spine", 7:"Tail"}

HOME_SPEED = 50
STRAIGHT_SPINE = 2048
STRAIGHT_SPEED = 50
STRAIGHT_SPINE_ARRAY = [STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE, STRAIGHT_SPINE]
STRAIGHT_SPEED_ARRAY = [STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED, STRAIGHT_SPEED]

# Selections corresponding to Addresses
# Selections originate from DisplayServoTraits Function
# Data Byte Sizes are Included Next to Trait Name
AddrDict = {                          #Byte Size
    1: 0, # Model Number                2
    2: 2, # Model Information           4
    3: 6, # Firmware Version            1
    4: 7, # ID                          1
    5: 8, # Baud Rate                   1
    6: 9, # Return Delay Time           1
    7: 10, # Drive Mode                 1
    8: 11, # Operating Mode             1
    9: 13, # Protocol Type              1
    10: 20, # Homing Offset             4
    11: 24, # Moving Threshold          4
    12: 31, # Temperature Limit         1
    13: 32, # Max Voltage Limit         2
    14: 34, # Min Voltage Limit         2
    15: 36, # PWM Limit                 2
    16: 38, # Current Limit             2
    17: 40, # Acceleration Limit        4
    18: 44, # Velocity Limit            4
    19: 48, # Max Position Limit        4
    20: 52, # Min Position Limit        4
    21: 63, # Shutdown                  1
    22: 64, # Torque Toggle             1
    23: 65, # LED                       1
    24: 68, # Status Return Level       1
    25: 69, # Registered Instruction    1
    26: 70, # Hardware Error Status     1
    27: 76, # Velocity I Gain           2
    28: 78, # Velocity P Gain           2
    29: 80, # Position D Gain           2
    30: 82, # Position I Gain           2
    31: 84, # Position P Gain           2
    32: 100, # Goal PWM                 2
    33: 102, # Goal Current             2
    34: 104, # Goal Velocity            4
    35: 108, # Profile Acceleration     4
    36: 112, # Profile Velocity         4
    37: 116, # Goal Position            4
    38: 120, # Realtime Tick            2
    39: 122, # Moving                   1
    40: 123, # Moving Status            1
    41: 124, # Present PWM              2
    42: 126, # Present Current          2
    43: 128, # Present Velocity         4
    44: 132, # Present Position         4
    45: 136, # Velocity Trajectory      4
    46: 140, # Position Trajectory      4
    47: 144, # Present Input Voltage    2   
    48: 146 # Present Temperature       1  
}

rows, cols = (22, 16) 

PinPulseTime = 2 # Amount of time a pin will be pulsed in seconds