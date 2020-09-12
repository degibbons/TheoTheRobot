#ALL MX-64(2.0) CONTROL TABLE ADDRESSES AND INITIAL VALUES
                                    # BYTE SIZE   #INITIAL VALUE
ADDR_MODEL_NUMBER           = 0             #2                   
ADDR_MODEL_INFORMATION      = 2             #4
ADDR_FIRMWARE_VERSION       = 6             #1
ADDR_ID                     = 7             #1
ADDR_BAUD_RATE              = 8             #1          1       # | 0=9,600 | 1(Default)=57,600 | 2=115,200 | 3=1M |
ADDR_RETURN_DELAY_TIME      = 9             #1          250
ADDR_DRIVE_MODE             = 10            #1          0       # |0=vELOCITY BASED| 1=TIME_BASED|
ADDR_OPERATING_MODE         = 11            #1          3       # |0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
                                                            #  1	Velocity Control Mode	This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
                                                            #  3    (Default)	Position Control Mode	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by Max Position Limit(48) and Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
                                                            #  4	Extended Position Control Mode(Multi-turn)	This mode controls position. This mode is identical to the Multi-Turn Mode from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear.
                                                            #  5	Current-based Position Control Mode	This mode controls both position and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers.
                                                            #  16	PWM Control Mode (Voltage Control Mode)	This mode directly controls PWM output. (Voltage Control Mode)
ADDR_PROTOCOL_TYPE          = 13            #1          2
ADDR_HOMING_OFFSET          = 20            #4          0       # Present Position(132) = Actual Position + Homing Offset(20)
ADDR_MOVING_THRESHOLD       = 24            #4          10      #ESSENTIALLY ACCURACY TOLERANCE
ADDR_TEMPERATURE_LIMIT      = 31            #1          80
ADDR_MAX_VOLTAGE_LIMIT      = 32            #2          160
ADDR_MIN_VOLTAGE_LIMIT      = 34            #2          95
ADDR_PWM_LIMIT              = 36            #2          885
ADDR_CURRENT_LIMIT          = 38            #2          1941
ADDR_ACCELERATION_LIMIT     = 40            #4          32767
ADDR_VELOCITY_LIMIT         = 44            #4          285             0-1023
ADDR_MAX_POSITION_LIMIT     = 48            #4          4095
ADDR_MIN_POSITION_LIMIT     = 52            #4          0
ADDR_SHUTDOWN               = 63            #1          52

#RAM AREA
ADDR_TORQUE_TOGGLE          = 64            #1          0
TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
ADDR_LED                    = 65            #1          0
ADDR_STATUS_RETURN_LEVEL    = 68            #1          2
ADDR_REGISTERED_INSTRUCTION = 69            #1          0
ADDR_HARDWARE_ERROR_STATUS  = 70            #1          0
ADDR_VELOCITY_I_GAIN        = 76            #2          1920
ADDR_VELOCITY_P_GAIN        = 78            #2          100
ADDR_POSITION_D_GAIN        = 80            #2          0
ADDR_POSITION_I_GAIN        = 82            #2          0
ADDR_POSITION_P_GAIN        = 84            #2          850

ADDR_GOAL_PWM               = 100           #2          
ADDR_GOAL_CURRENT           = 102           #2
ADDR_GOAL_VELOCITY          = 104           #4                  #ONLY IN VELOCITY/WHEEL MODES
ADDR_PROFILE_ACCELERATION   = 108           #4          0
ADDR_PROFILE_VELOCITY       = 112           #4          0       #THIS IS THE IMPORTANT ONE
ADDR_GOAL_POSITION          = 116           #4          
ADDR_REALTIME_TICK          = 120           #2
ADDR_MOVING                 = 122           #1          0
ADDR_MOVING_STATUS          = 123           #1          0
ADDR_PRESENT_PWM            = 124           #2
ADDR_PRESENT_CURRENT        = 126           #2
ADDR_PRESENT_VELOCITY       = 128           #4          
ADDR_PRESENT_POSITION       = 132           #4
ADDR_VELOCITY_TRAJECTORY    = 136           #4                  #Desired Velocity Trajectory from Profile
ADDR_POSITION_TRAJECTORY    = 140           #4                  #Desired Position Trajectory from Profile
ADDR_PRESENT_INPUT_VOLTAGE  = 144           #2
ADDR_PRESENT_TEMPERATURE    = 146           #1