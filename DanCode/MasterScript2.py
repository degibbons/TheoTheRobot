# Ping Servos to See Which Ones Are Connected

# Assign Servo Object To Each One

# Assemble Limb Objects Using Present Servos


import os
import numpy as np
import time
from ControlTable import *
from RelevantFunctions import *
from dynamixel_sdk import *
from threading import Thread

dxl_data_list = PingServos()
for dxl_id in dxl_data_list:
    print("[ID:%03d] Detected" % (dxl_id))
    
