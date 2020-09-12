import numpy as np
import pandas as pd
import math
import time
import copy as cp
import sys

#df = pd.read_csv('KinematicsTiming3.csv')



#tspan = float(input("What timespan in seconds do you want one stride to take?\n"))

def calculateSpeeds(tspan,positionsFile):


    df = pd.read_csv(positionsFile)

    df = df.values[:][:]
    cf = cp.copy(df)

    h_stance = 4.892994
    h_swing = 3.347006
    f_stance = 5.211718
    f_swing = 3.028282
    h_st_per = h_stance / 8.24
    h_sw_per = h_swing / 8.24
    f_st_per = f_stance / 8.24
    f_sw_per = f_swing / 8.24
    b = cf.shape
    cTotal = b[0]
    cLength = cTotal//2
    cWidth = b[1]
    percents = np.linspace(1,cLength-1,cLength-1)
    joints = list(range(1,cWidth+1))
    percents = percents.astype(int).tolist()
    distTravel = np.zeros((11,16))

    for i in percents:
        for j in joints:
            if (j == 1 or j == 2 or j == 3 or j == 4 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*f_st_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 5 or j == 6 or j == 7 or j == 8): 
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*f_sw_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 9 or j == 10 or j == 11 or j == 12 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*h_st_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4
            elif (j == 13 or j == 14 or j == 15 or j == 16 ):
                T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
                distTravel[i-1,j-1] = T0
                T2 = (tspan*h_sw_per/cLength)/60
                T3 = (T0 / T2) / 0.114
                T4 = round(T3,5)
                cf[i-1][j-1] = T4

    cf = np.round(cf)
    joints = list(range(0,cWidth))

    for j in joints:
        if (j == 0 or j == 1 or j == 2 or j == 3):
            R1 = abs(df[-1][j] - df[0][j+4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * f_st_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 4 or j == 5 or j == 6 or j == 7):
            R1 = abs(df[-1][j] - df[0][j-4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * f_sw_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 8 or j == 9 or j == 10 or j == 11):
            R1 = abs(df[-1][j] - df[0][j+4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * h_st_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)
        elif (j == 12 or j == 13 or j == 14 or j == 15):
            R1 = abs(df[-1][j] - df[0][j-4])/4096
            distTravel[-1,j] = R1
            R2 = (tspan * h_sw_per / cLength) / 60
            R3 = (R1 / R2) / 0.114
            cf[-1][j] = round(R3)

    cf[cf==0]=1
    rows = cf.shape[0]
    cols = cf.shape[1]

    for x in range(0, rows):
        for y in range(0, cols):
            if (cf[x][y] > 1023):
                cf[x][y] = 1023

    efa = distTravel[:,(0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11)]
    efb = distTravel[:,(4,5,6,7,4,5,6,7,12,13,14,15,12,13,14,15)]
    distances = np.concatenate((efa,efb),axis=0)

    for p in [0,1,2,3]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-1)
        distances[:,p] = efc1
        
    for p in [4,5,6,7]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-9)
        distances[:,p] = efc1

    for p in [8,9,10,11]:
        efc1 = distances[:,p]
        efc1 = np.roll(efc1,-8)
        distances[:,p] = efc1

    cfa = cf[:,(0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11)]
    cfb = cf[:,(4,5,6,7,4,5,6,7,12,13,14,15,12,13,14,15)]
    cfc = np.concatenate((cfa,cfb),axis=0)

    for q in [0,1,2,3]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-1)
        cfc[:,q] = cfc1
        
    for q in [4,5,6,7]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-9)
        cfc[:,q] = cfc1

    for q in [8,9,10,11]:
        cfc1 = cfc[:,q]
        cfc1 = np.roll(cfc1,-8)
        cfc[:,q] = cfc1

    speeds = np.multiply(cfc,.114)
    distances[distances==0]=.0000001
    ansFinal = np.divide(distances,speeds)
    slowest = []

    for k in range(0,22):
        movement = ansFinal[k][:]
        temp1 = movement.max()
        movement = movement.tolist()
        temp2 = movement.index(temp1)
        slowest.append(temp2)

    slowestServos = [x + 1 for x in slowest]

    slowestFR = []

    FR_slwst = [0] * 22
    FL_slwst = [0] * 22
    BR_slwst = [0] * 22
    BL_slwst = [0] * 22

    for k in range(0,22):
        movement = ansFinal[k][0:3]
        temp1 = movement.max()
        movement = movement.tolist()
        temp2 = movement.index(temp1)
        slowestFR.append(temp2)

    slowestServosFR = [x + 1 for x in slowestFR]

    for m in list(range(0,22)):
        if(slowestServos[m]==1):
            FR_slwst[m] = 1
        elif(slowestServos[m]==2): 
            FR_slwst[m] = 2
        elif(slowestServos[m]==3): 
            FR_slwst[m] = 3
        elif(slowestServos[m]==4): 
            FR_slwst[m] = 4
        elif(slowestServos[m]==5):
            FL_slwst[m] = 5
        elif(slowestServos[m]==6): 
            FL_slwst[m] = 6
        elif(slowestServos[m]==7):
            FL_slwst[m] = 7
        elif(slowestServos[m]==8): 
            FL_slwst[m] = 8
        elif (slowestServos[m]==9): 
            BR_slwst[m] = 9
        elif(slowestServos[m]==10): 
            BR_slwst[m] = 10
        elif(slowestServos[m]==11): 
            BR_slwst[m] = 11
        elif(slowestServos[m]==12): 
            BR_slwst[m] = 12
        elif (slowestServos[m]==13): 
            BL_slwst[m] = 13
        elif(slowestServos[m]==14): 
            BL_slwst[m] = 14
        elif(slowestServos[m]==15): 
            BL_slwst[m] = 15
        elif(slowestServos[m]==16): 
            BL_slwst[m] = 16
        
    for p in list(range(0,22)):
        print(str(FR_slwst[p]),str(FL_slwst[p]),str(BR_slwst[p]),str(BL_slwst[p]))

