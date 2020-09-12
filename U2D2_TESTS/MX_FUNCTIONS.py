import os
import time
from dynamixel_sdk      import *                    # Uses Dynamixel SDK library
from MX_IDS_ETC_VALUES  import *
from MX_CONTROL_TABLE   import *
number_of_servos_connected = 3
inter_assign_time_delay = 0.1
def INITIAL_SETUP():
    
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
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port:  \t"+ DEVICENAME)
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
        
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate: %03d BPS" %(BAUDRATE))
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set drive mode to vel based
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_DRIVE_MODE, DRIVE_MODE_VEL_BASED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Drive mode set to: %03d" %(DXL_ID[index], DRIVE_MODE_VEL_BASED))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set operating mode to joint/position control mode
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_OPERATING_MODE, OPERATING_JOINT_POSITION_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Operating mode set to: %03d" %(DXL_ID[index], OPERATING_JOINT_POSITION_MODE))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set velocity limit 
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_H)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Velocity limit set to: %03d" %(DXL_ID[index], VELOCITY_LIMIT_H))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set acceleration limit 
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_ACCELERATION_LIMIT, ACCELERATION_LIMIT_M)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Acceleration limit set to: %03d" %(DXL_ID[index], ACCELERATION_LIMIT_M))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set max position limit
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Max position limit set to: %03d" %(DXL_ID[index], MAX_POSITION_LIMIT))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set min position limit
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Min position limit set to: %03d" %(DXL_ID[index], MIN_POSITION_LIMIT))
        index += 1

    time.sleep(inter_assign_time_delay)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set moving threshold
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_MOVING_THRESHOLD, MOVING_THRESHOLD_ACCURACY_H)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Moving accuracy set to high: %03d" %(DXL_ID[index], MOVING_THRESHOLD_ACCURACY_H))
        index += 1

    time.sleep(inter_assign_time_delay)

    print("\n\tSetup Completed\n")


def MOVE_HOME_ALL():
    print("\n\tMoving Home\n")
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

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        #print("Succeeded to open the port")
        pass
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
        
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        #print("Succeeded to change the baudrate")
        pass
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Initialize GroupSyncWrite instance
    groupSyncWrite_HOME_POSN = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4)

    # Initialize GroupSyncRead instace for Present Position, vel
    groupSyncRead_PRES_POSN = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4)

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set velocity limit 
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_H)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        #else:
            #print("[ID:%03d] Velocity limit set to: %03d" %(DXL_ID[index], VELOCITY_LIMIT_H))
            
        index += 1


    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        #Set profile vel  
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[index], ADDR_PROFILE_VELOCITY, THEO_HOME_PROF_VEL)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] Profile Velocity set to: %03d" %(DXL_ID[index], THEO_HOME_PROF_VEL))
        index += 1


    index   = 0
    while 1:
        if index == number_of_servos_connected:
            break
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_result = groupSyncRead_PRES_POSN.addParam(DXL_ID[index])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead_PRES_POSN addparam failed" % DXL_ID[index])
            quit()
        index   += 1


    param_home_goal_position = [DXL_LOBYTE(DXL_LOWORD(THEO_GENERIC_HOME_POSITION)), DXL_HIBYTE(DXL_LOWORD(THEO_GENERIC_HOME_POSITION)), DXL_LOBYTE(DXL_HIWORD(THEO_GENERIC_HOME_POSITION)), DXL_HIBYTE(DXL_HIWORD(THEO_GENERIC_HOME_POSITION))]

    index   = 0
    while 1:
        if index == number_of_servos_connected:
            break        
        dxl_addparam_result = groupSyncWrite_HOME_POSN.addParam(DXL_ID[index], param_home_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite_HOME_POSN addparam failed" % DXL_ID[index])
            quit()
        index   += 1
        

    index   = 0
    while 1:
        if index == number_of_servos_connected:
            break
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_TORQUE_TOGGLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque on" % DXL_ID[index])
        index   += 1

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite_HOME_POSN.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite_HOME_POSN.clearParam()

    while 1:
         # Syncread present position
        dxl_comm_result = groupSyncRead_PRES_POSN.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        index   = 0
        while 1:
            if index == number_of_servos_connected:
                break
            # Check if groupsyncread data of Dynamixel is available
            dxl_getdata_result = groupSyncRead_PRES_POSN.isAvailable(DXL_ID[index], ADDR_PRESENT_POSITION, 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID[index])
                quit()
            index   +=  1
        
        if number_of_servos_connected == 2:
            dxl1_present_position = groupSyncRead_PRES_POSN.getData(DXL1_ID, ADDR_PRESENT_POSITION, 4)
            dxl2_present_position = groupSyncRead_PRES_POSN.getData(DXL2_ID, ADDR_PRESENT_POSITION, 4)
            print("GoalPos:%03d   [ID:%03d]   PresPos:%03d\t[ID:%03d]  PresPos:%03d" % (THEO_GENERIC_HOME_POSITION, DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
            if not (abs(THEO_GENERIC_HOME_POSITION - dxl1_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                if not (abs(THEO_GENERIC_HOME_POSITION - dxl2_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                    break
        elif number_of_servos_connected == 4:
            dxl1_present_position = groupSyncRead_PRES_POSN.getData(DXL1_ID, ADDR_PRESENT_POSITION, 4)
            dxl2_present_position = groupSyncRead_PRES_POSN.getData(DXL2_ID, ADDR_PRESENT_POSITION, 4)
            dxl3_present_position = groupSyncRead_PRES_POSN.getData(DXL3_ID, ADDR_PRESENT_POSITION, 4)
            dxl4_present_position = groupSyncRead_PRES_POSN.getData(DXL4_ID, ADDR_PRESENT_POSITION, 4)
            print("GoalPos:%03d   [ID:%03d]   PresPos:%03d\t[ID:%03d]  PresPos:%03d\t[ID:%03d]  PresPos:%03d\t[ID:%03d]  PresPos:%03d" % (THEO_GENERIC_HOME_POSITION, DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position, DXL3_ID, dxl3_present_position, DXL4_ID, dxl4_present_position))
            if not (abs(THEO_GENERIC_HOME_POSITION - dxl1_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                if not (abs(THEO_GENERIC_HOME_POSITION - dxl2_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                    if not (abs(THEO_GENERIC_HOME_POSITION - dxl3_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                        if not (abs(THEO_GENERIC_HOME_POSITION - dxl4_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                            break
        elif number_of_servos_connected == 24:
            dxl1_present_position = groupSyncRead_PRES_POSN.getData(DXL1_ID, ADDR_PRESENT_POSITION, 4)
            dxl5_present_position = groupSyncRead_PRES_POSN.getData(DXL5_ID, ADDR_PRESENT_POSITION, 4)
            dxl9_present_position = groupSyncRead_PRES_POSN.getData(DXL9_ID, ADDR_PRESENT_POSITION, 4)
            dxl13_present_position = groupSyncRead_PRES_POSN.getData(DXL13_ID, ADDR_PRESENT_POSITION, 4)
            print("GoalPos:%03d   [ID:%03d]   PresPos:%03d\t[ID:%03d]  PresPos:%03d\t[ID:%03d]  PresPos:%03d\t[ID:%03d]  PresPos:%03d" % (THEO_GENERIC_HOME_POSITION, DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position, DXL3_ID, dxl3_present_position, DXL4_ID, dxl4_present_position))
            if not (abs(THEO_GENERIC_HOME_POSITION - dxl1_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                if not (abs(THEO_GENERIC_HOME_POSITION - dxl5_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                    if not (abs(THEO_GENERIC_HOME_POSITION - dxl9_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                        if not (abs(THEO_GENERIC_HOME_POSITION - dxl13_present_position) > HOME_MOVING_STATUS_THRESHOLD):
                            break

    print("\n\tSet to home position. \n")
    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_TORQUE_TOGGLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque off" % DXL_ID[index])
        index   += 1
    time.sleep(1)

def MX_MOVE_SINGLE(S_ID, POSITION, VELOCITY):
    print("\n\tStarting move function\n")
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

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        pass
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
        
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        pass
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    MX_ID = S_ID 
    GOAL_POSITION_MX = POSITION
    PROFILE_VEL = VELOCITY   
    check_id = 0
    check_goal_pos = 0
    check_profile_vel = 0

    if MX_ID > 24:
        print("ID ERROR, DOES NOT EXIST!!!\n")
        check_id = 1
    elif GOAL_POSITION_MX > MAX_POSITION_LIMIT or GOAL_POSITION_MX < MIN_POSITION_LIMIT:
        print("POSITION ERROR, POSITION OUT OF RANGE!!!\n")
        check_goal_pos = 1
    elif PROFILE_VEL > VELOCITY_LIMIT_H or PROFILE_VEL < VELOCITY_LIMIT_L:
        print("VELOCITY ERROR, VELOCITY OUT OF RANGE!!!\n")
        check_profile_vel = 1
        

    if check_id == 0:
        if check_goal_pos == 0:
            if check_profile_vel == 0:
                #check if torque is on
                dxl_torque_status, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, MX_ID, ADDR_TORQUE_TOGGLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                
                if dxl_torque_status == 1:
                    print("Torque already on.\n")
                elif dxl_torque_status == 0:
                    print("Torque off, turning on.\n")
                    # Enable Dynamixel Torque
                    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MX_ID, ADDR_TORQUE_TOGGLE, TORQUE_ENABLE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    else:
                        print("Dynamixel#%d torque ON\n" % MX_ID)
                
                # Write profile vel
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, MX_ID, ADDR_PROFILE_VELOCITY, PROFILE_VEL)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Profile Velocity set to: %03d\n" %(PROFILE_VEL))

                # Write goal position  
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, MX_ID, ADDR_GOAL_POSITION, GOAL_POSITION_MX)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Goal Position set to: %03d\n" %(GOAL_POSITION_MX))

                while 1: 
                    mx_i_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, MX_ID, ADDR_PRESENT_POSITION)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    
                    mx_i_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, MX_ID, ADDR_PRESENT_VELOCITY)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    mx_i_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, MX_ID, ADDR_PRESENT_CURRENT)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    mx_i_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, MX_ID, ADDR_PRESENT_INPUT_VOLTAGE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                    mx_i_present_voltage = mx_i_present_voltage * .1
                    
                    mx_i_present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, MX_ID, ADDR_PRESENT_TEMPERATURE)
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    print("GoalPos:%03d   [ID:%03d]   PresPos:%03d\t    PresVel:%03d\t    PresCurrent:%03d\t    PresVoltage:%03d\t    PresTemp:%03d\t" %(GOAL_POSITION_MX, MX_ID, mx_i_present_position, mx_i_present_velocity, mx_i_present_current, mx_i_present_voltage, mx_i_present_temp))
                    if abs(mx_i_present_position - GOAL_POSITION_MX) <  MOVING_THRESHOLD_ACCURACY_M:
                        print("[ID:%03d] Reached Position: %03d\n" %(MX_ID, mx_i_present_position))
                        break

    elif check_id == 1 or check_goal_pos ==1 or check_profile_vel == 1:
        print("\tFatal Error\n")
    
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MX_ID, ADDR_TORQUE_TOGGLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque off" % MX_ID)

def MX_MOVE_LIMB(LIMB, POSN_ARRAY, VEL_ARRAY):
    print("\n\tStarting move LIMB function\n")
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

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        #print("Succeeded to open the port")
        pass
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
        
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        #print("Succeeded to change the baudrate")
        pass
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    #THERE AR 21 POSN, VEL POINTS FOR THIS TEST

    if LIMB == "F_R_ARM":
        F_R_ARM = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID]
        index_ID_LIM = len(F_R_ARM)
    elif LIMB == "F_ARM_2_TEST":
        F_R_ARM_2_TEST = [DXL1_ID, DXL2_ID]
        index_ID_LIM = len(F_R_ARM_2_TEST)
    
    # Initialize GroupBulkWrite instance
    groupBulkWrite_PROFILE_VEL = GroupBulkWrite(portHandler, packetHandler)
    groupBulkWrite_GOAL_POSN = GroupBulkWrite(portHandler, packetHandler)

    index = 0
    while 1:
        if index == index_ID_LIM:
            break
        # enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_TORQUE_TOGGLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque on" % DXL_ID[index])
        index   += 1


  

    index_col = 0
    while 1:
        if index_col == 21:
            break
        


        index = 0
        while 1:
            if index == index_ID_LIM:
                index = 0
                break

            param_goal_velocity_ = [DXL_LOBYTE(DXL_LOWORD(VEL_ARRAY[DXL_ID[index]-1][index_col])), DXL_HIBYTE(DXL_LOWORD(VEL_ARRAY[DXL_ID[index]-1][index_col])), DXL_LOBYTE(DXL_HIWORD(VEL_ARRAY[DXL_ID[index]-1][index_col])), DXL_HIBYTE(DXL_HIWORD(VEL_ARRAY[DXL_ID[index]-1][index_col]))]
            param_goal_pos_ = [DXL_LOBYTE(DXL_LOWORD(POSN_ARRAY[DXL_ID[index]-1][index_col])), DXL_HIBYTE(DXL_LOWORD(POSN_ARRAY[DXL_ID[index]-1][index_col])), DXL_LOBYTE(DXL_HIWORD(POSN_ARRAY[DXL_ID[index]-1][index_col])), DXL_HIBYTE(DXL_HIWORD(POSN_ARRAY[DXL_ID[index]-1][index_col]))]
            
            # Add Dynamixel#1 profile velocity value to the Bulkwrite parameter storage
            dxl_addparam_result = groupBulkWrite_PROFILE_VEL.addParam(DXL_ID[index], ADDR_PROFILE_VELOCITY, 4, param_goal_velocity_)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % DXL_ID[index])
                quit()

            # Add Dynamixel#2 profile velocity value to the Bulkwrite parameter storage
            dxl_addparam_result = groupBulkWrite_GOAL_POSN.addParam(DXL_ID[index], ADDR_GOAL_POSITION, 4, param_goal_pos_)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % DXL_ID[index])
                quit()
            
            index += 1

        # Bulkwrite goal profile velocities
        dxl_comm_result = groupBulkWrite_PROFILE_VEL.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        groupBulkWrite_PROFILE_VEL.clearParam()

        # Bulkwrite goal profile velocities
        dxl_comm_result = groupBulkWrite_GOAL_POSN.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        groupBulkWrite_GOAL_POSN.clearParam()



        print("Moving step: %03d" %(index_col))

        time.sleep(.1)
        index_col += 1

    index = 0
    while 1:
        if index == number_of_servos_connected:
            break
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[index], ADDR_TORQUE_TOGGLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque off" % DXL_ID[index])
        index   += 1
