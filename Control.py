import math
from BallInterface import BallInterface

import threading
import time

import numpy as np
import paho.mqtt.client as mqtt
import random


class Control:

    def __init__(self, controlledSystem, broker='seazero.local'):
        self.__initMQTT(broker)
        self.__initOpenLoop()
        self.__initPID()
        self.__initLQR()
        self.__initKalman()
        self.__controlledSystem = controlledSystem
        self.ref = 0.0 
        self.T = 0.018
        

    def __initMQTT(self, broker='seazero.local'):
        self.__broker = broker
        self.__mqttClient = mqtt.Client('LuciÂ­a')
        self.__mqttClient.connect(broker, 1883)
        self.__mqttClient.loop_start()


    def __initOpenLoop(self):
        self.__runningOL = False


    def __initPID(self):
        self.__runningPID = True
        self.Kp = 5
        self.Td = 0.3
        self.Ti = 3
        self.N = 10


    def __initLQR(self):
        self.__runningLQR = True
        self.Kt = [3.5289876, 1.3912153]
        self.Kr = 3.5289876


    def __initKalman(self):
        self.__runningKalman = True
        self.A = np.array([[0, 1], [0, -0.7800118875890534]])
        self.B = np.array([[0],[0.5894041732678656]])
        self.Ct = np.array([[1, 0]])
        self.L = np.array([[0.4490204],[0.0508097]])


    def runOpenLoop(self):
        with open('openLoopData.txt', 'w') as file:
            pass
        while self.__runningOL:
            controlValue = self.Kp*self.ref
            print(controlValue)
            self.__controlledSystem.writeControlValue(controlValue)
            y = self.__controlledSystem.getY()
            with open('openLoopData.txt', 'a') as file:
                file.write(str(time.time()) + ',' +  str(self.ref) + ',' + str(y) + '\n')
            self.__publishPosition(y)
    

    def runPID(self):
        with open('PIDData.txt', 'w') as file:
            pass
        lastTime = time.time()
        lastErr = 0.0
        errSum = 0.0
        refPrev = self.ref
        lastFilter = 0.0
        while self.__runningPID:
            if refPrev != self.ref:
                errSum = 0
            refPrev = self.ref

            now = time.time()
            timeChange = now - lastTime

            y = self.__controlledSystem.getY()
            error = self.ref - y

            errSum += (error * timeChange)
            filter = math.exp(-self.N*self.T/self.Td)*lastFilter + self.N*error - self.N*lastErr

            controlValue = self.Kp * (error + errSum/self.Ti + filter)

            self.__controlledSystem.writeControlValue(controlValue)

            self.__publishPosition(errSum)
            with open('PIDData.txt', 'a') as file:
                file.write(str(time.time()) + ',' +  str(self.ref) + ',' + str(y) + '\n')

            lastErr = error
            lastTime = now
            lastFilter = filter


    def runLQR(self):
        with open('LQRData.txt', 'w') as file:
            pass
        yList = [self.__controlledSystem.getY()]*10
        lastTime = time.time()
        lasty = self.__controlledSystem.getY()
        lastFilter = 0.0
        i = 0
        errSum = 0.0
        refPrev = self.ref
        while self.__runningLQR:
            if refPrev != self.ref:
                errSum = 0
            refPrev = self.ref

            now = time.time()
            timeChange = now - lastTime

            y = self.__controlledSystem.getY() 
            error = self.ref - y
            errSum += (error * timeChange) 
            filter = math.exp(-self.N*self.T/1)*lastFilter + self.N*y - self.N*lasty

            feedback = y*self.Kt[0] + filter*self.Kt[1]
            controlValue = self.Kr*self.ref + self.Kr*errSum/self.Ti  - feedback
            self.__controlledSystem.writeControlValue(controlValue)

            with open('LQRData.txt', 'a') as file:
                file.write(str(time.time()) + ',' +  str(self.ref) + ',' + str(y) + '\n')

            lastTime = now
            lasty = y
            lastFilter = filter


    def runKalman(self):
        with open('KalmanData.txt', 'w') as file:
            pass
        lastxObs = np.array([[0],[0]])
        lastu = 0
        lasty = self.__controlledSystem.getY()
        while self.__runningKalman:
            y = self.__controlledSystem.getY()

            AminusLCt = self.A - np.dot(self.L,self.Ct)
            BtimesU = np.dot(self.B, lastu)
            LtimesY = np.dot(self.L, lasty) 
            dxObs = np.add(np.add(np.dot(AminusLCt, lastxObs), BtimesU), LtimesY)
            xObs = lastxObs + dxObs*self.T
  
            feedback = xObs[0][0]*self.Kt[0] + xObs[1][0]*self.Kt[1]
            u = (self.Kr*self.ref) - feedback
            self.__controlledSystem.writeControlValue(u)
            with open('KalmanData.txt', 'a') as file:
                file.write(str(time.time()) + ',' +  str(self.ref) + ',' + str(y) + '\n')        

            lastxObs = xObs
            lastu = u
            lasty = y
    


    def runSquare(self):
        threadDynamic = threading.Thread(target=self.runPID, args=())
        threadDynamic.start()
        threadDynamic2 = threading.Thread(target=self.runOpenLoop, args=())

        self.ref = 250
        time.sleep(10)

        self.__runningPID = False
        self.__runningOL = True
        threadDynamic2.start()

        initialTime = time.time()
        while True:
            self.ref = self.__getSquare(100, 0, 2, time.time()-initialTime)


    def runDeadZoneUp(self):
        value = 0
        for i in range(46):
            self.__runningPID = True
            threadDynamic = threading.Thread(target=self.runPID, args=())
            threadDynamic.start()
            self.ref = 100
            time.sleep(15)
            self.__runningPID = False
            threadDynamic.join()

            value += 100
            self.__controlledSystem.writeControlValue(value)
            t0 = time.time()
            while self.__controlledSystem.getY() < 400:
                self.__controlledSystem.writeControlValue(value)
                pass

            self.__publishString(str(time.time() - t0) + "," + str(value))
            print(str(time.time() - t0) + "," + str(value))


    def runDeadZoneDown(self):
        value = 100
        for i in range(40):
            self.__runningPID = True
            threadDynamic = threading.Thread(target=self.runPID, args=())
            threadDynamic.start()
            self.ref = 400
            time.sleep(15)
            self.__runningPID = False
            threadDynamic.join()

            value -= 100
            self.__controlledSystem.writeControlValue(value)
            t0 = time.time()
            while self.__controlledSystem.getY() > 100:
                self.__controlledSystem.writeControlValue(value)
                

            self.__publishString(str(time.time() - t0) + "," + str(value))
            print(str(time.time() - t0) + "," + str(value))


    def __getSin(self, amplitude, center, w, now):
        return math.sin(w * now) * amplitude + center


    def __getSquare(self, amplitude, center, w, now):
        sin = math.sin(w * now) * amplitude
        if sin > 0:
            return amplitude + center
        else:
            return -amplitude + center


    def __publishPosition(self, position):
        self.__mqttClient.publish('ball/measures/position', str(position))


    def __publishString(self, message):
        self.__mqttClient.publish('ball/measures/string', message)


    def __subscribePosition(self):
        self.__mqttClient.subscribe("ball/parameters/reference")
        self.__mqttClient.on_message = self.__changePosition


    def __subscribePID(self):
        self.__mqttClient.subscribe("ball/parameters/PID")
        self.__mqttClient.on_message = self.changePID


    def __changePosition(self, client, userdata, message):
        self.ref = int(message.payload.decode("utf-8"))


    def runBall(self):
        threadDynamic = threading.Thread(target=self.runPID, args=())
        threadDynamic.start()


if __name__ == "Control":
    mb = BallInterface()
    mc = Control(mb)
    mc.runBall()