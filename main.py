import cv2
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
from PID import PID

class interIIT():
	def __init__(self):
		self.defVisOdoConst()
		self.defSensorConst()
		self.defPIDinstance()
		self.defMotorSpeed()

		self.clearSensorReadParams()
		self.clearMotorWriteParams()
		self.clearVOparams()

		self.startProcess()

	def defVisOdoConst(self):
		self.VOsift = cv2.xfeatures2d.SIFT_create()
		self.VObf = cv2.BFMatcher()

	def defSensorConst(self):
		self.ssColorValues = {
			"red":{"acc":[255,0,0],"lower":[200,-1,-1],"upper":[260,40,40]},
			"yellow":{"acc":[255,255,0],"lower":[200,200,-1],"upper":[260,260,40]},
			} # just putting values, check reading and update

	def defPIDinstance(self):
		self.PID = PID(P=0.2,I=0.1,D=0.3)

	def defMotorSpeed(self):
		self.speed["front"] = {"left": 200,"right": 200}
		self.speed["back"]  = {"left":-200,"right":-200}
		self.speed["left"]  = {"left":-200,"right": 200}
		self.speed["right"] = {"left": 200,"right":-200}

		self.ActAngle = {"high":{"front":0,"back":0},"low":{"front":90,"back":90}}

	def clearSensorReadParams(self):
		self.ssUSpidRead = [0.0,0.0]
		self.ssUSfrontRead = 0.0
		self.ssColorRead = {}
		self.ssColorRead["front"] = [0,0,0]
		self.ssColorRead["back"] = [0,0,0]
		self.ssMPUangleRead = 0.0
		self.ssReadBreak = False
		self.ssWriteBreak = False

	def clearVOparams(self):
		self.VOpos = 0.0
		self.VObreak = False

		self.safeStartDis = 10
		self.startSeedDis = 30
		self.blockEndDis = 200
		self.safeStopDis = 10

		self.disWallClimb = 5

	def clearMotorWriteParams(self):
		self.speedMotor = {"left":0,"right":0}
		self.actuatorValue = {"front":0,"back":0}

		self.restAngle = 0
		self.levelAngle = -30
		self.ploughAngle = 30

		self.farmServo = self.restAngle

		self.seedingAngle = 90
		self.seedRestAngle = 0

		self.seedServo = self.seedRestAngle

		self.harvestServo = False

	def comA2R(self):
		# to be changed
		# serial communication from arduino to Rpi
		while True:
			self.ssUSpidRead = [0,0]
			self.ssUSfrontRead = 0
			self.ssColorFrontRead = [0,0,0]
			self.ssColorBackRead = [0,0,0]
			self.ssMPUangleRead = 0.0
			if self.ssReadBreak:
				break

	def comR2A(self):
		# serial communication from Rpi to arduino
		# writing angle of all 4 motors
		while:
			if self.ssWriteBreak:
				break

	def VisOdo(self):
		self.VOpos 

	def startProcess(self):
		# parallelly updating sensor reading through serial communication
		self.threadComA2R = threading.Thread(target=self.comA2R,args=())
		self.threadComA2R.start()
		self.threadComR2A = threading.Thread(target=self.comR2A,args=())
		self.threadComR2A.start()

		# assuming - only 2 stairs with lower level for harvesting and upper for ploughing, seeding and levelling 

		# start from red zone and go till red zone is in visible range in back color sensor
		self.PID.clear()
		self.stopMotor()
		while(self.isSameColor("red",self.ssColorRead["back"])):
			self.goForward()

		self.stopMotor()
		self.clearVisOdo()
		self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
		self.threadVisOdo.start()

		self.startHarvesting()
		while(self.VOpos<self.blockEndDis):
			self.goForward()

		self.stopMotor()
		self.PID.clear()
		self.VObreak = True
		self.threadVisOdo.join()

		self.stopMotor()

		while not(self.isSameColor("red",self.ssColorRead["front"])):
			self.goBackward()

		self.stopLevelling()

		self.clearVisOdo()
		self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
		self.threadVisOdo.start()

		while(self.VOpos<self.safeStopDis):
			self.goBackward()

		self.stopMotor()
		self.PID.clear()
		self.VObreak = True
		self.threadVisOdo.join()

		self.climbingAlgo()

		self.PID.clear()
		self.stopMotor()
		while(self.isSameColor("red",self.ssColorRead["back"])):
			self.goForward()

		self.stopMotor()
		self.clearVisOdo()
		self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
		self.threadVisOdo.start()

		while(self.VOpos<self.safeStartDis):
			self.goForward()

		self.stopMotor()

		self.startPloughing()
		while(self.VOpos<self.blockEndDis):
			self.goForward()

		self.stopMotor()
		self.PID.clear()

		while(self.VOpos>(self.blockEndDis-self.startSeedDis)):
			self.goBackward()

		self.VObreak = True
		self.threadVisOdo.join()

		self.stopMotor()
		self.startSeeding()
		self.startLevelling()

		while not(self.isSameColor("red",self.ssColorRead["back"])):
			self.goBackward()

		self.stopMotor()
		self.stopSeeding()

		while not(self.isSameColor("red",self.ssColorRead["front"])):
			self.goBackward()

		self.stopLevelling()

		self.clearVisOdo()
		self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
		self.threadVisOdo.start()

		while(self.VOpos<self.safeStopDis):
			self.goBackward()

		self.stopMotor()
		self.PID.clear()
		self.VObreak = True
		self.threadVisOdo.join()

	def climbingAlgo(self):
		angle = self.ssMPUangleRead
		while(angle - 90 < self.ssMPUangleRead):
			self.rotateLeft()
		self.stopMotor()

		while(self.ssUSfrontRead > self.disWallClimb):
			self.goForward(withPID=False)
		self.stopMotor()

		self.setActuator(front="high",back="high")

		nowTime = time.time()
		while(time.time()-nowTime<2):
			self.goForward(withPID=False)
		self.stopMotor()

		self.setActuator(front="low",back="high")

		nowTime = time.time()
		while(time.time()-nowTime<5):
			self.goForward(withPID=False)
		self.stopMotor()

		self.setActuator(front="low",back="low")

		nowTime = time.time()
		while(time.time()-nowTime<5):
			self.goForward(withPID=False)
		self.stopMotor()

		angle = self.ssMPUangleRead
		while(angle + 90 > self.ssMPUangleRead):
			self.rotateRight()
		self.stopMotor()

	def setActuator(self,front,back):
		self.actuatorValue["front"] = self.ActAngle[front]["front"]
		self.actuatorValue["back"]  = self.ActAngle[back]["back"]

	def goForward(self,withPID=True):
		self.goBot("front",withPID)

	def goBackward(self,withPID=True):
		self.goBot("back",withPID)

	def goBot(self,goDir,withPID=True):
		if(withPID):
			output = self.PID.update(np.avg(self.ssUSpidRead))
		else:
			output = 0
		self.setMotorSpeed(baseSpeed=self.speed[goDir],pid=output)

	def rotateLeft(self):
		self.setMotorSpeed(baseSpeed=self.speed["left"],pid=0)

	def rotateRight(self):
		self.setMotorSpeed(baseSpeed=self.speed["right"],pid=0)

	def stopMotor(self):
		self.speedMotor = {"left":0,"right":0}

	def setMotorSpeed(self,baseSpeed,pid):
		self.speedMotor["left"]  = baseSpeed - pid # check sign
		self.speedMotor["right"] = baseSpeed + pid # sign : negative of left
		# no change in actuator angles

	def startPloughing(self):
		self.farmServo = self.ploughAngle

	def stopPloughing(self):
		self.farmServo = self.restAngle

	def startSeeding(self):
		self.seedServo = self.seedingAngle

	def stopSeeding(self):
		self.seedServo = self.seedRestAngle

	def startLevelling(self):
		self.farmServo = self.levelAngle

	def stopLevelling(self):
		self.farmServo = self.restAngle

	def startHarvesting(self):
		self.harvestServo = True

	def stopHarvesting(self):
		self.harvestServo = False

	def isSameColor(self,colorCode,readColor):
		dictColor = self.ssColorValues
		for i in range(3):
			if not(dictColor["lower"][i]<=readColor[i]):
				return False
			if not(dictColor["upper"][i]>=readColor[i]):
				return False
		return True
