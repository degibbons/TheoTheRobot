# Control table address and Values
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_VELOCITY_LIMIT         = 44            #4
ADDR_DRIVE_MODE             = 10            #1          0       # |0=vELOCITY BASED| 1=TIME_BASED|
DRIVE_MODE_VEL_BASED        = 0            #establish default drive mode, time or vel based
ADDR_OPERATING_MODE         = 11            #1          3       # |0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
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

PreferedDelay               = 0.01

# Control table address
ADDR_PRO_BAUDRATE           = 8                             # Control table address is different in Dynamixel model

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Positions File Name
PositionsFile = "KinematicsTiming4.csv"

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
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

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

FileExist = 0       #Indicates if the file data is written to exists or not

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

## Hindlimb Stance Right
HL_ST_R_1 = [1443, 1480, 1541, 1652, 1786, 1901, 2136, 2234, 2422, 2494, 2499]
HL_ST_R_2 = [1802, 1867, 1883, 1998, 2009, 1891, 1997, 1991, 2009, 2043, 2046]
HL_ST_R_3 = [2121, 1971, 1899, 1819, 1767, 1680, 1676, 1544, 1503, 1594, 1661]
HL_ST_R_4 = [2687, 2551, 2377, 2131, 1981, 1941, 1839, 1924, 2096, 2279, 2369]
## Hindlimb Swing Right
HL_SW_R_1 = [2500, 2638, 2806, 2893, 2893, 2759, 2407, 1913, 1593, 1438, 1427]
HL_SW_R_2 = [2045, 1989, 1975, 2022, 2077, 2077, 2137, 2389, 2223, 1885, 1855]
HL_SW_R_3 = [1660, 1297, 1171, 1225, 1234, 1136, 1136, 1378, 1737, 2039, 1978]
HL_SW_R_4 = [2380, 2736, 2835, 2791, 2785, 2547, 2546, 2467, 2519, 2573, 2620]
## Hindlimb Total Stride Right
HL_TOT_R_1 = HL_ST_R_1 + HL_SW_R_1
HL_TOT_R_2 = HL_ST_R_2 + HL_SW_R_2
HL_TOT_R_3 = HL_ST_R_3 + HL_SW_R_3
HL_TOT_R_4 = HL_ST_R_4 + HL_SW_R_4

## Hindlimb Stance Left
HL_ST_L_1 = [2653, 2616, 2555, 2444, 2310, 2195, 1960, 1862, 1674, 1602, 1597]
HL_ST_L_2 = [1802, 1867, 1883, 1998, 2009, 1891, 1997, 1991, 2009, 2043, 2046]
HL_ST_L_3 = [1975, 2125, 2197, 2277, 2329, 2416, 2420, 2552, 2593, 2502, 2435]
HL_ST_L_4 = [2687, 2551, 2377, 2131, 1981, 1941, 1839, 1924, 2096, 2279, 2369]
## Hindlimb Swing Left
HL_SW_L_1 = [1596, 1458, 1290, 1203, 1203, 1337, 1689, 2183, 2503, 2658, 2669]
HL_SW_L_2 = [2045, 1989, 1975, 2022, 2077, 2077, 2137, 2389, 2223, 1885, 1855]
HL_SW_L_3 = [2436, 2799, 2925, 2871, 2862, 2960, 2960, 2718, 2359, 2057, 2118]
HL_SW_L_4 = [2380, 2736, 2835, 2791, 2785, 2547, 2456, 2467, 2519, 2573, 2620]
## Hindlimb Total Stride Left
HL_TOT_L_1 = HL_ST_L_1 + HL_SW_L_1
HL_TOT_L_2 = HL_ST_L_2 + HL_SW_L_2
HL_TOT_L_3 = HL_ST_L_3 + HL_SW_L_3
HL_TOT_L_4 = HL_ST_L_4 + HL_SW_L_4

## Forelimb Stance Right
FL_ST_R_1 = [1680, 1838, 2001, 1998, 1990, 1981, 1975, 2008, 2127, 2177, 2197]
FL_ST_R_2 = [2115, 2081, 2045, 2048, 2058, 2078, 2109, 2191, 2315, 2305, 2295]
FL_ST_R_3 = [2338, 2401, 2467, 2451, 2407, 2345, 2277, 2204, 2068, 1811, 1653]
FL_ST_R_4 = [2153, 2031, 1894, 1880, 1877, 1876, 1869, 1731, 1424, 1632, 1818]
## Forelimb Swing Right
FL_SW_R_1 = [2197, 2227, 2288, 2329, 2307, 2220, 2092, 1949, 1816, 1718, 1680]
FL_SW_R_2 = [2295, 2293, 2265, 2230, 2220, 2238, 2263, 2272, 2226, 2153, 2115]
FL_SW_R_3 = [1653, 1563, 1392, 1282, 1323, 1468, 1675, 1906, 2120, 2277, 2338]
FL_SW_R_4 = [1818, 1839, 1897, 1980, 2076, 2235, 2411, 2474, 2377, 2229, 2153]
## Forelimb Total Stride Right
FL_TOT_R_1 = FL_ST_R_1 + FL_SW_R_1
FL_TOT_R_2 = FL_ST_R_2 + FL_SW_R_2
FL_TOT_R_3 = FL_ST_R_3 + FL_SW_R_3
FL_TOT_R_4 = FL_ST_R_4 + FL_SW_R_4

## Forelimb Stance Left
FL_ST_L_1 = [2416, 2258, 2095, 2098, 2106, 2115, 2121, 2088, 1969, 1919, 1899]
FL_ST_L_2 = [2115, 2081, 2045, 2048, 2058, 2078, 2109, 2191, 2315, 2305, 2295]
FL_ST_L_3 = [1758, 1695, 1629, 1645, 1689, 1751, 1819, 1892, 2028, 2285, 2443]
FL_ST_L_4 = [2153, 2031, 1894, 1880, 1877, 1876, 1869, 1731, 1424, 1632, 1818]
## Forelimb Swing Left
FL_SW_L_1 = [1899, 1869, 1808, 1767, 1789, 1876, 2004, 2147, 2280, 2378, 2416]
FL_SW_L_2 = [2295, 2293, 2265, 2230, 2220, 2238, 2263, 2272, 2226, 2153, 2115]
FL_SW_L_3 = [2443, 2533, 2704, 2814, 2773, 2628, 2421, 2190, 1976, 1819, 1758]
FL_SW_L_4 = [1818, 1839, 1897, 1980, 2076, 2235, 2411, 2474, 2377, 2229, 2153]
## Forelimb Total Stride Left
FL_TOT_L_1 = FL_ST_L_1 + FL_SW_L_1
FL_TOT_L_2 = FL_ST_L_2 + FL_SW_L_2
FL_TOT_L_3 = FL_ST_L_3 + FL_SW_L_3
FL_TOT_L_4 = FL_ST_L_4 + FL_SW_L_4


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