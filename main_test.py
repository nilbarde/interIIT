# import cv2
import numpy as np
#import matplotlib.pyplot as plt
import threading
import time
import RPi.GPIO as GPIO
from PID import PID
import serial

GPIO.setmode(GPIO.BOARD)

class interIIT():
    def __init__(self):
        self.defSetGpioPins()
        #self.defVisOdoConst()
        self.defStepperConst()
        self.defSensorConst()
        self.defPIDinstance()
        self.defStepClimbConst()
        self.defMotorSpeed()
        self.defSerComConst()

        self.clearSensorReadParams()
        self.clearMotorWriteParams()
        self.clearVOparams()

        self.startProcess()
        while(True):
            self.ssReadColor()

    def defSetGpioPins(self):
        self.pinsUSpidEchos = {"pid1":3,"pid2":8,"pid3":7}
        self.pinsUSpidTrigs = {"pid1":5,"pid2":10,"pid3":11}
        self.pinsUSfrontEcho = 16
        self.pinsUSfrontTrig = 18
        for i in self.pinsUSpidEchos:
            GPIO.setup(self.pinsUSpidEchos[i], GPIO.IN)
            GPIO.setup(self.pinsUSpidTrigs[i], GPIO.OUT)
            GPIO.output(self.pinsUSpidTrigs[i], False)
        GPIO.setup(self.pinsUSfrontEcho, GPIO.IN)
        GPIO.setup(self.pinsUSfrontTrig, GPIO.OUT)
        GPIO.output(self.pinsUSfrontTrig, False)
        
        self.pinsColorS3 = {"front":36,"back":22}
        self.pinsColorS2 = {"front":38,"back":24}
        self.pinsColorRead = {"front":40,"back":26}
        
        for i in self.pinsColorRead:
            GPIO.setup(self.pinsColorS2[i], GPIO.OUT)
            GPIO.setup(self.pinsColorS3[i], GPIO.OUT)
            GPIO.setup(self.pinsColorRead[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #def defVisOdoConst(self):
        # self.VOsift = cv2.xfeatures2d.SIFT_create()
        # self.VObf = cv2.BFMatcher()

    def defStepperConst(self):
        self.servoStepAngle = 1.8

    def defSensorConst(self):
        self.ssColorValues = {
            "red":{"acc":[255,0,0],"lower":[200,-1,-1],"upper":[260,40,40]},
            "yellow":{"acc":[255,255,0],"lower":[200,200,-1],"upper":[260,260,40]},
            } # just putting values, check reading and update
        self.ssUSpidRead = {"pid1":0.0,"pid2":0.0,"pid3":0.0}
        self.ssUSfrontRead = 0.0
        self.ssColorRead = {
            "front":[0,0,0],
            "back":[0,0,0],
            }
        self.ssColorNumCycle = 50

    def defPIDinstance(self):
        self.PID = +PID(P=0.4,I=0.1,D=0.3)

    def defMotorSpeed(self):
        self.speed = {}
        self.speed["front"] = {"left": 5,"right": 5}
        self.speed["back"]  = {"left":-5,"right":-5}
        self.speed["left"]  = {"left":-5,"right": 5}
        self.speed["right"] = {"left": 5,"right":-5}

        self.ActAngle = {"high":{"front":0,"back":0},"low":{"front":90,"back":90}}

    def defSerComConst(self):
        self.messenger = serial.Serial('/dev/ttyACM1', 9600)

    def defStepClimbConst(self):
        #decide exactly what should be the distance while stepping down
        self.stepDdisWallW1 =  40
        self.stepDdisWallW2 =  80
        self.stepDdisWallW3 = 110
        self.stepDdisWallSafe = 15

        self.stepUdisWallW1 = 10
        self.stepUdisWallW2 = 80
        self.stepUdisWallW3 = 40
        self.stepUdisWallSafe = 15

    def clearSensorReadParams(self):
        self.ssUSpidRead = {"pid1":0.0,"pid2":0.0,"pid3":0.0}
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

    def VisOdo(self):
        self.VOpos 

    def ssRead(self):
        while self.ReadingSensor:
            self.ssUSpidRead = {"pid1":0.0,"pid2":0.0,"pid3":0.0}
            self.ssUSfrontRead = 0.0
            self.ssColorFrontRead = [0,0,0]
            self.ssColorBackRead = [0,0,0]

            # self.ssReadUS()
            # self.ssReadColor()
            print("good")
            time.sleep(1)

    def ssReadUS(self):
        for pin in self.ssUSpidRead:
            GPIO.output(self.pinsUSpidTrigs[pin], True)
            time.sleep(0.00001)
            GPIO.output(self.pinsUSpidTrigs[pin], False)

            startTime = time.time()
            while GPIO.input(self.pinsUSpidEchos[pin]) == 0:
                startTime = time.time()
            # save time of arrival
            stopTime = time.time()
            while GPIO.input(self.pinsUSpidEchos[pin]) == 1:
                stopTime = time.time()

            timeElapsed = stopTime - startTime

            self.ssUSpidRead[pin] = timeElapsed*17150 + 1.15
            print(self.ssUSpidRead[pin],"  ",pin)

        GPIO.output(self.pinsUSfrontTrig, True)
        time.sleep(0.00001)
        GPIO.output(self.pinsUSfrontTrig, False)

        startTime = time.time()
        while GPIO.input(self.pinsUSfrontEcho) == 0:
            startTime = time.time()
        # save time of arrival
        stopTime = time.time()
        while GPIO.input(self.pinsUSfrontEcho) == 1:
            stopTime = time.time()

        timeElapsed = stopTime - startTime

        self.ssUSfrontRead = timeElapsed*17150 + 1.15
        print(self.ssUSfrontRead,"  ","front")


    def ssReadColor(self):
        for pin in ["front"]:
            GPIO.output(self.pinsColorS2[pin],False)
            GPIO.output(self.pinsColorS3[pin],False)
            time.sleep(0.1)
            start = time.time()
            for _ in range (self.ssColorNumCycle):
                GPIO.wait_for_edge(self.pinsColorRead[pin], GPIO.FALLING)
            duration = time.time() - start
            self.ssColorRead[pin][0] = self.ssColorNumCycle/duration

            GPIO.output(self.pinsColorS2[pin],False)
            GPIO.output(self.pinsColorS3[pin],True)
            time.sleep(0.3)
            start = time.time()
            for _ in range (self.ssColorNumCycle):
                GPIO.wait_for_edge(self.pinsColorRead[pin], GPIO.FALLING)
            duration = time.time() - start
            self.ssColorRead[pin][1] = self.ssColorNumCycle/duration

            GPIO.output(self.pinsColorS2[pin],True)
            GPIO.output(self.pinsColorS3[pin],False)
            time.sleep(0.3)
            start = time.time()
            for _ in range (self.ssColorNumCycle):
                GPIO.wait_for_edge(self.pinsColorRead[pin], GPIO.FALLING)
            duration = time.time() - start
            self.ssColorRead[pin][2] = self.ssColorNumCycle/duration
            print(self.ssColorRead[pin])

    def startProcess(self):
        # parallelly updating sensor reading through serial communication
        # self.threadComA2R = threading.Thread(target=self.comA2R,args=())
        # self.threadComA2R.start()
        # self.threadComR2A = threading.Thread(target=self.comR2A,args=())
        # self.threadComR2A.start()

        self.ReadingSensor = True
        self.threadssRead = threading.Thread(target=self.ssRead,args=())
        self.threadssRead.start()
        self.rootMPUangle = self.ssMPUangleRead
        # assuming - only 2 stairs with lower level for harvesting and upper for ploughing, seeding and levelling 

    def otherProcess(self):
        # start from red zone and go till red zone is in visible range in back color sensor
        self.PID.clear()
        self.stopMotor()
        while(self.isSameColor("red",self.ssColorRead["back"])):
            self.goForward()

        self.stopMotor()
        self.clearVisOdo()
        # self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
        # self.threadVisOdo.start()

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
        # self.threadVisOdo.join()

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
        # self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
        # self.threadVisOdo.start()

        while(self.VOpos<self.safeStopDis):
            self.goBackward()

        self.stopMotor()
        self.PID.clear()
        self.VObreak = True
        # self.threadVisOdo.join()

        self.climbingDownAlgo()
    
        self.PID.clear()
        self.stopMotor()
        while(self.isSameColor("red",self.ssColorRead["back"])):
            self.goForward()

        self.stopMotor()
        self.clearVisOdo()
        # self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
        # self.threadVisOdo.start()

        self.startHarvesting()
        while(self.VOpos<self.blockEndDis):
            self.goForward()

        self.stopMotor()
        self.PID.clear()
        self.VObreak = True
        # self.threadVisOdo.join()

        self.stopMotor()
        self.stopHarvesting()

        while not(self.isSameColor("red",self.ssColorRead["front"])):
            self.goBackward()

        self.stopLevelling()

        self.clearVisOdo()
        # self.threadVisOdo = threading.Thread(target=self.VisOdo,args=())
        # self.threadVisOdo.start()

        while(self.VOpos<self.safeStopDis):
            self.goBackward()

        self.stopMotor()
        self.PID.clear()
        self.VObreak = True
        # self.threadVisOdo.join()

        self.climbingUpAlgo()

    def climbingUpAlgo(self):
        angle = self.ssMPUangleRead
        while(angle - 90 < self.ssMPUangleRead):
            self.rotateLeft()
        self.stopMotor()

        while(self.ssUSfrontRead > self.stepUdisWallW1):
            self.goForward(withPID=False)
        self.stopMotor()

        self.setActuator(front="high",back="high")

        while(self.ssUSfrontRead > self.stepUdisWallW2):
            self.goForward(withPID=False)
        self.stopMotor()

        self.setActuator(front="low",back="high")

        while(self.ssUSfrontRead > self.stepUdisWallW3):
            self.goForward(withPID=False)
        self.stopMotor()

        self.setActuator(front="low",back="low")

        while(self.ssUSfrontRead > self.stepUdisWallSafe):
            self.goForward(withPID=False)
        self.stopMotor()

    def climbingDownAlgo(self):
        self.rotateLeft()
        self.stopMotor()

        while(self.ssUSfrontRead < self.stepDdisWallW1):
            self.goBackward(withPID=False)
        self.stopMotor()

        self.setActuator(front="low",back="high")

        while(self.ssUSfrontRead < self.stepDdisWallW2):
            self.goBackward(withPID=False)
        self.stopMotor()

        self.setActuator(front="high",back="high")

        while(self.ssUSfrontRead < self.stepDdisWallW3):
            self.goBackward(withPID=False)
        self.stopMotor()

        self.setActuator(front="low",back="low")

        while(self.ssUSfrontRead < self.stepDdisWallSafe):
            self.goBackward(withPID=False)
        self.stopMotor()

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
            angle = self.ssMPUangleRead - self.rootMPUangle
        else:
            output = 0
            angle = 0
        self.setMotorSpeed(baseSpeed=self.speed[goDir],pid=output,angle=angle)

    def setMotorSpeed(self,baseSpeed,pid,angle):
        # self.speedMotor["left"]  = baseSpeed - pid - angle*10 # check sign
        # self.speedMotor["right"] = baseSpeed + pid + angle*10 # sign : negative of left
        # no change in actuator angles
        leftSpeed  = baseSpeed - pid - angle/10
        rightSpeed = baseSpeed  + pid + angle/10
        self.goMotor(int(leftSpeed),int(rightSpeed))

    def sendSerial(self,code):
        # x = (chr(code)).encode()
        self.messenger.write(chr(code))

    def goMotor(self,leftSpeed,rightSpeed):
        leftSteps = abs(leftSpeed)
        rightSteps = abs(rightSpeed)
        leftDir = 2*(0<leftSpeed)-1
        rightDir = 2*(0<rightSpeed)-1
        noSteps = max(leftSteps,rightSteps)
        code = 0
        for i in range(noSteps):
            x = leftSteps>i
            code += x*(1==leftDir)
            code += x*(-1==leftDir)*16
            x = rightSteps>i
            code += x*(1==rightDir)*2
            code += x*(-1==rightDir)*32
            self.sendSerial(code)

    def rotateLeft(self):
        self.setRotate(dir=+1)

    def rotateRight(self):
        self.setRotate(Dir=-1)

    def setRotate(self,Dir):
        if(Dir==1):
            self.sendSerial(18)
        elif(Dir==1):
            self.sendSerial(33)

    def stopMotor(self):
        self.speedMotor = {"left":0,"right":0}

    def setFarmServo(self,angle):
        if(self.farmServo<angle):
            self.farmServo += self.servoStepAngle
            # send write via serial communication
        elif(self.farmServo>angle):
            self.farmServo -= self.servoStepAngle
            # send write via serial communication

    def startPloughing(self):
        self.setFarmServo(self.ploughAngle)

    def stopPloughing(self):
        self.setFarmServo(self.restAngle)

    def startSeeding(self):
        self.setFarmServo(self.seedingAngle)

    def stopSeeding(self):
        self.setFarmServo(self.seedRestAngle)

    def startLevelling(self):
        self.setFarmServo(self.levelAngle)

    def stopLevelling(self):
        self.setFarmServo(self.restAngle)

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

x = interIIT()