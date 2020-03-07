"""
Created on 3/5/20
Marquette Robotics Club
Danny Hudetz
Purpose: do various shared functions between 2d and 3d visualizers
"""

import numpy as math
import h5py as hdf

class backEnd:

    def __init__(self, a,b,c):
        self.a=a
        self.b=b
        self.c=c

    def calculateComponents(self, t1, t2, t3):
        ta = math.deg2rad(t1)
        ar = self.a*math.cos(ta)
        az = -self.a*math.sin(ta)

        tb = math.deg2rad(t2-270+t1)
        br = self.b*math.sin(tb)
        bz = self.b*math.cos(tb)

        tc = math.deg2rad(t3-(90-(t2-180+t1)))
        cr = self.c*math.sin(tc)
        cz = self.c*math.cos(tc)
        return (ar,az,br,bz,cr,cz)

def getServoAngles(file, requestedRadius, requestedZ):
    zStrings = list(file.keys())
    zFloats = []
    for z in zStrings:
        zFloats.append(float(z))
    lastDiff = 1000000.0 #this can be any arbitrary value
    for z in zFloats:
        if requestedZ == z:
            foundZ = z
        else:
            absoluteDiff = float(abs(requestedZ - z))
            if lastDiff > absoluteDiff:
                lastDiff = absoluteDiff
                foundZ = z
    foundZ = str(foundZ)
    rStrings = file[foundZ]
    rFloats = []
    for r in rStrings:
        rFloats.append(float(r))
    lastDiff = 1000000.0 #this can also be any arbitrary value
    for r in rFloats:
        if requestedRadius == r:
            foundR = r
        else:
            absoluteDiff = float(abs(requestedRadius - r))
            if lastDiff > absoluteDiff:
                lastDiff = absoluteDiff
                foundR = r
    foundR = str(foundR)
    print("Found R: ", foundR)
    print("Found Z: ", foundZ)
    servoAngles = rStrings[foundR]
    print(servoAngles[1], servoAngles[2], servoAngles[3])
    return (servoAngles[1], servoAngles[2], servoAngles[3])

def getFile(fileName):
    a=b=c=0
    f=None
    try:
        f = hdf.File("simulated\\"+fileName, "r")
        periodSplit=fileName.split('.')
        periodSplit.pop()
        valueSplit=".".join(periodSplit).split('-')
        a=float(valueSplit[0])
        b=float(valueSplit[1])
        c=float(valueSplit[2])

        print("File successfully imported.")
    except:
        print("Check if file exists. Makes sure to inlude \'.hdf5\'")
    return f,a,b,c


def getSineMovement(previousAngles,requestedAngles):
    smoothAngles=[]
    for a in math.arange(0.0,math.pi,math.pi/100):
        angleSet=[]
        for i in range(len(previousAngles)):
            angleSet.append(((previousAngles[i]-requestedAngles[i])*(math.cos(a)+1))/2+requestedAngles[i])
        smoothAngles.append(angleSet)
    return smoothAngles
