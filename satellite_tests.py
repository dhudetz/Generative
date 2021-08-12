from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
import os
from panda3d.core import LineSegs, NodePath, MeshDrawer
import numpy as np

class Planet(ShowBase):

	def __init__(self):
		ShowBase.__init__(self)

		#Set background color of scene (dark purple)
		base.setBackgroundColor(0.1,0,0.2,1.0)

		#Define the globe model
		self.globe = self.loader.loadModel(os.getcwd() + "/models/Icosahedron.egg")

		# Reparent the model to render.
		self.globe.reparentTo(self.render)

		#Set color, scale, and position of globe
		self.globe.setScale(0.5,0.5,0.5)
		self.globe.setPos(0,0,0)
		self.globe.setColor(0.05,0.5,0.2,1.0)
		
		#Define the satellite properties
		self.numSats = 5
		self.satellites = np.array([], dtype=float)
		
		#Create and draw each path
		for i in range(self.numSats):
			sat = self.createSatPath(10,0.0,float(i+1)*0.5)
			np.append(self.satellites, sat)
			

		# Add the spinCameraTask procedure to the task manager.
		self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")


	def createSatPath(self, polarAngle, tiltAngle, radius, numSteps = 40):
		ls = LineSegs()		

		polarAngle = np.deg2rad(polarAngle)
		tiltAngle = np.deg2rad(tiltAngle)

		for i in range(numSteps + 1):
			a = polarAngle * i
			y = sin(a)*radius
			x = cos(a)*radius
			ls.drawTo(x, 0, y)
			print(ls.getCurrentPosition())
		satNode = NodePath(ls.create())
		satNode.reparentTo(self.render)
		return satNode

	# Define a procedure to move the camera.
	def spinCameraTask(self, task):
		angleDegrees = task.time * 15
		angleRadians = angleDegrees * (pi / 180.0)
		self.camera.setPos(10 * sin(angleRadians), -10 * cos(angleRadians), 0)
		self.camera.setHpr(angleDegrees, 0, 0)
		return Task.cont
app = Planet()
app.run()


#NOTES

#TEXTURES:
	#tex = loader.loadTexture('streetscene_env.jpg')
	#teapot.setTexGen(TextureStage.getDefault(), TexGenAttrib.MEyeSphereMap)
	#teapot.setTexture(tex)
	
#MESH DRAWER (not sure what it does):
		#Define MeshDrawer to draw sat paths
		#self.meshDrawer = MeshDrawer()
		#self.meshDrawer.setBudget(1000)
		#meshDrawerNode = self.meshDrawer.getRoot()
		#meshDrawerNode.reparentTo(self.render)
		#meshDrawerNode.setDepthWrite(False)
		
