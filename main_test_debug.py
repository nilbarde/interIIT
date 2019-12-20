# import cv2
import numpy as np
#import matplotlib.pyplot as plt
import threading
import time
import RPi.GPIO as GPIO
from PID import PID
import serial
import math
import smbus

# GPIO.setmode(GPIO.BOARD)

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
        # while(True):
        #     self.ssReadColor()

    def defSetGpioPins(self):
        self.pinsUSpidEchos = {"pid1":3,"pid2":8,"pid3":7}
        self.pinsUSpidTrigs = {"pid1":5,"pid2":10,"pid3":11}
        self.pinsUSfrontEcho = 16
        self.pinsUSfrontTrig = 18
        return 
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
        self.wheelRadius = 2.5
        self.stepperMove = (math.pi*self.wheelRadius)/180.0

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

        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47

        self.bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
        self.Device_Address = 0x68   # MPU6050 device address

        self.MPU_Init()

    def MPU_Init(self):
        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
        
        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
        
        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
        
        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self,addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    def defPIDinstance(self):
        self.PID = PID(P=0.4,I=0.1,D=0.3)

    def defMotorSpeed(self):
        self.speed = {}
        self.speed["front"] = {"left": 5,"right": 5}
        self.speed["back"]  = {"left":-5,"right":-5}
        self.speed["left"]  = {"left":-5,"right": 5}
        self.speed["right"] = {"left": 5,"right":-5}

        self.ActAngle = {"high":{"front":0,"back":0},"low":{"front":90,"back":90}}

    def defSerComConst(self):
        return
        self.messenger = serial.Serial('/dev/ttyACM1', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

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
        self.blockEndDis = 10
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

            self.ssReadUS()
            self.ssReadColor()
            self.ssReadMPU()

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

    def ssReadMPU(self):
        self.gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        self.gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        self.gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        self.ssMPUangleRead = self.gyro_z

    def startProcess(self):
        self.ReadingSensor = True
        self.threadssRead = threading.Thread(target=self.ssRead,args=())
        self.threadssRead.start()
        self.rootMPUangle = self.ssMPUangleRead
        # assuming - only 2 stairs with lower level for harvesting and upper for ploughing, seeding and levelling 

        # start from red zone and go till red zone is in visible range in back color sensor
        self.goPlough()

        self.goSeed()

        self.goDown()

        self.goHarvest()

        self.goBack()

        self.goUp()

    def goPlough(self):
        self.PID.clear()
        while(self.isSameColor("red",self.ssColorRead["back"])):
            self.goForward()

        self.clearVOparams()

        while(self.VOpos<self.safeStartDis):
            self.goForward()

        self.startPloughing()
        while(self.ssUSfrontRead>self.blockEndDis):
            self.goForward()

        self.PID.clear()

    def goSeed(self):
        while(self.ssUSfrontRead<(self.blockEndDis+self.startSeedDis)):
            self.goBackward()

        self.startSeeding()
        self.startLevelling()

        while not(self.isSameColor("red",self.ssColorRead["back"])):
            self.goBackward()

        self.stopSeeding()

        while not(self.isSameColor("red",self.ssColorRead["front"])):
            self.goBackward()

        self.stopLevelling()

        self.clearVOparams()
        while(self.VOpos<self.safeStopDis):
            self.goBackward()

        self.PID.clear()
        self.clearVOparams()

    def goDown(self):
        self.climbingDownAlgo()
    
    def goHarvest(self):
        self.PID.clear()
        while(self.isSameColor("yellow",self.ssColorRead["back"])):
            self.goForward()

        self.clearVOparams()

        self.startHarvesting()
        while(self.VOpos<self.blockEndDis):
            self.goForward()

        self.PID.clear()

        self.stopHarvesting()

    def goBack(self):
        while not(self.isSameColor("yellow",self.ssColorRead["front"])):
            self.goBackward()

        self.clearVOparams()

        while(self.VOpos<self.safeStopDis):
            self.goBackward()

        self.stopMotor()
        self.PID.clear()
        self.VObreak = True

    def goUp(self):
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
            output = 0
            for US in self.ssUSpidRead:
                output += self.ssUSpidRead[US]
            output /= len(self.ssUSpidRead)
            angle = self.ssMPUangleRead - self.rootMPUangle
        else:
            output = 0
            angle = 0
        self.setMotorSpeed(baseSpeed=self.speed[goDir],pid=output,angle=angle)

    def setMotorSpeed(self,baseSpeed,pid,angle):
        # self.speedMotor["left"]  = baseSpeed - pid - angle*10 # check sign
        # self.speedMotor["right"] = baseSpeed + pid + angle*10 # sign : negative of left
        # no change in actuator angles
        leftSpeed  = baseSpeed["left"]  - pid - angle/10
        rightSpeed = baseSpeed["right"] + pid + angle/10
        self.goMotor(int(leftSpeed),int(rightSpeed))

    def sendSerial(self,code):
        # x = (chr(code)).encode()
        print(code)
        return
        self.messenger.write(chr(code))

    def goMotor(self,leftSpeed,rightSpeed):
        leftSteps = abs(leftSpeed)
        rightSteps = abs(rightSpeed)
        leftDir = 2*(0<leftSpeed)-1
        rightDir = 2*(0<rightSpeed)-1
        noSteps = max(leftSteps,rightSteps)
        self.VOpos += self.stepperMove*noSteps
        for i in range(noSteps):
            code = 0
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
            while(self.farmServo<angle):
                self.farmServo += self.servoStepAngle
                # send write via serial communication
                self.sendSerial(17)
        elif(self.farmServo>angle):
            while(self.farmServo>angle):
                self.farmServo -= self.servoStepAngle
                # send write via serial communication
                self.sendSerial(34)

    def startPloughing(self):
        print("startPloughing")
        self.setFarmServo(self.ploughAngle)

    def stopPloughing(self):
        print("stopPloughing")
        self.setFarmServo(self.restAngle)

    def startSeeding(self):
        print("startSeeding")
        self.setFarmServo(self.seedingAngle)

    def stopSeeding(self):
        print("stopSeeding")
        self.setFarmServo(self.seedRestAngle)

    def startLevelling(self):
        print("startLevelling")
        self.setFarmServo(self.levelAngle)

    def stopLevelling(self):
        print("stopLevelling")
        self.setFarmServo(self.restAngle)

    def startHarvesting(self):
        print("startHarvesting")
        self.sendSerial(68)
        self.harvestServo = True

    def stopHarvesting(self):
        print("stopHarvesting")
        self.sendSerial(136)
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