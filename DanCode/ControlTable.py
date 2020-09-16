# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_VELOCITY_LIMIT         = 44            #4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Positions File Name
PositionsFile = "KinematicsTiming3.xlsx"

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

TORQUE_ON               = 1                 # Value for enabling the torque
TORQUE_OFF              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_VELOCITY_LIMIT          = 4

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