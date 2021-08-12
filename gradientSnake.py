"""
Created on 3/5/20
Marquette Robotics Club
Danny Hudetz
Purpose: read from the hdf5 format and visualize the coordinates mapped nearest
         to user input in 3 dimensions
"""

from math import pi, sin, cos
from numpy import deg2rad
from random import random
import vis
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from pandac.PandaModules import *
from panda3d.core import *
from direct.interval.IntervalGlobal import *
import threading
import sys,os

class visualizer(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)
        w, h = 750, 750
        self.thickness=50

        props = WindowProperties()
        props.setSize(w, h)

        mydir = os.path.abspath(sys.path[0])
        self.mydir = Filename.fromOsSpecific(mydir).getFullpath()

        base.win.requestProperties(props)
        base.setBackgroundColor(0,0,0)
        self.a=self.b=self.c=self.counter=0
        self.center = LVector3f(0,0,0)
        self.back=vis.backEnd(self.a,self.b,self.c)
        self.aSeg = LineSegs("a")
        self.bSeg = LineSegs("b")
        self.cSeg = LineSegs("c")
        self.aSeg.setColor(1, 1 ,1 ,1)
        self.bSeg.setColor(1, 1 ,1 ,1)
        self.cSeg.setColor(1, 1 ,1 ,1)
        self.segments=(self.aSeg,self.bSeg,self.cSeg)
        self.models=[]
        self.modelNum=0
        self.previousColor=(0,0,0)
        self.currentColor=(random(),random(),random())
        self.colorForward=True
        for s in self.segments:
            s.setThickness(self.thickness)
        self.segmentNodes=[]
        for s in self.segments:
            self.segmentNodes.append(s.create(False))
        #grid drawing
        tileSize=10
        numLines=100
        doGrid=False
        if doGrid:
            for x in range(int(-numLines/2),int(numLines/2)):
                gridSegment=LineSegs("g")
                gridSegment.setColor(0.3,0.3,0.3,1)
                gridSegment.setThickness(5)
                gridSegment.drawTo(x*tileSize, (-numLines/2)*tileSize, 0)
                gridSegment.drawTo(x*tileSize, (numLines/2)*tileSize, 0)
                render.attachNewNode(gridSegment.create(False))
            for y in range(int(-numLines/2),int(numLines/2)):
                gridSegment=LineSegs("g")
                gridSegment.setColor(0.3,0.3,0.3,1)
                gridSegment.setThickness(5)
                gridSegment.drawTo((-numLines/2)*tileSize, y*tileSize, 0)
                gridSegment.drawTo((numLines/2)*tileSize, y*tileSize, 0)
                render.attachNewNode(gridSegment.create(False))
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

    def changeSegments(self,a,b,c):
        self.a=a
        self.b=b
        self.c=c
        self.back=vis.backEnd(self.a,self.b,self.c)


    def drawSegments(self, t0, t1, t2, t3):
        numModels=500
        if self.modelNum==numModels:
            self.modelNum=0
            self.previousColor=self.currentColor
            self.currentColor=(random(),random(),random())
        for sNode in self.segmentNodes:
            np=NodePath(sNode)
            np.removeNode()
        if len(self.models)>numModels:
            self.models[0].removeNode()
            self.models.pop(0)
        (ar,az,br,bz,cr,cz)=self.back.calculateComponents(t1,t2,t3)
        aVector = LVector3f(ar,0,az)
        bVector = LVector3f(br,0,bz)
        cVector = LVector3f(cr,0,cz)
        POI = self.center+aVector+bVector+cVector
        vertices=[self.center,self.center+aVector,self.center+aVector+bVector,POI]
        rotatedVertices=[]
        for vertex in vertices:
            q=Quat()
            q.set_from_axis_angle(t0+180, LVector3f(0,0,1))
            rotatedVertices.append(q.xform(vertex))
        drawSegs=False
        if drawSegs:
            self.aSeg.drawTo(rotatedVertices[0])
            self.aSeg.drawTo(rotatedVertices[1])
            self.bSeg.drawTo(rotatedVertices[1])
            self.bSeg.drawTo(rotatedVertices[2])
            self.cSeg.drawTo(rotatedVertices[2])
            self.cSeg.drawTo(rotatedVertices[3])
        self.segmentNodes=[]
        for s in self.segments:
            self.segmentNodes.append(s.create(False))
        #for vertex in rotatedVertices:
        modelIso = loader.loadModel(self.mydir + "/models/Icosahedron.egg")
        self.models.append(modelIso)
        finalColorValues=[]
        for i in range(0,3):
            finalColorValues.append(abs(self.previousColor[i]*(1-self.modelNum/numModels)-self.currentColor[i]*(self.modelNum/numModels))*0.8+0.2)
        modelIso.setColor(finalColorValues[0],finalColorValues[1],finalColorValues[2],1)
        modelIso.setPos(rotatedVertices[3])
        modelIso.setScale(5,5,5)
        modelIso.reparentTo(self.render)
        for sNode in self.segmentNodes:
            render.attachNewNode(sNode)
        self.modelNum+=1

    def spinCameraTask(self, task):
        angleDegrees = task.time * 10.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(200 * sin(angleRadians), -200 * cos(angleRadians), -200)
        self.camera.setHpr(angleDegrees, 45, 180)
        return Task.cont

    def generationTask(self, task):
        currentFrame = task.frame
        self.degrees+=0.9
        while currentFrame<10000:
            c0=1.3
            c1=0.23
            c2=0.525
            c3=0.7921
            self.drawSegments(c0*self.degrees,c1*self.degrees,c2*self.degrees,c3*self.degrees)
            return Task.cont

    def newGeneration(self):
        self.degrees=0
        self.taskMgr.add(self.generationTask, "generationTask")

    def close(self):
        sys.exit()

app = visualizer()
done=False

def userInputLoop():
    global done, currentAngles, previousAngles, back, app
    print("\nGenerative Visualizer")
    while not done:
        userInput = input("q to quit, go to generate: ")
        words=userInput.split()
        if words[0] == 'q':
            done=True
        elif words[0] == 'go':
            app.newGeneration()

try:
    userThread = threading.Thread(target=userInputLoop, args=())
    userThread.start()
except:
    print("Error: unable to start thread")

app.changeSegments(2,25,25)
app.run()
