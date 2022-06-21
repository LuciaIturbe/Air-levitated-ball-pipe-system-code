import VL53L0X
from rpi_hardware_pwm import HardwarePWM

import board
import busio
import adafruit_pca9685

import random


class BallInterface:
    
    def __init__(self, tubeHeight=583):
        self.__tubeHeight = tubeHeight
        self.__initPWM()
        self.__initBonnet()
        self.__initSensor()
        self.__initPredistortion()
    

    def __initPWM(self):
        self.__PWM = HardwarePWM(pwm_channel=0, hz=50)
        self.__PWM.change_frequency(10000)
        self.__PWM.start(16)

    
    def __initBonnet(self):
        self.__hat = adafruit_pca9685.PCA9685(busio.I2C(board.SCL, board.SDA))
        self.__hat.frequency = 1600
        self.__PWMchannel = self.__hat.channels[0]
        self.__PWMchannel.duty_cycle = 0

    
    def __initSensor(self):
        self.__sensor = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        self.__sensor.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)


    def __initPredistortion(self, filename="predistortionCSV.txt"):
        self.__predistortionDictionary = dict()
        with open(filename, 'r') as file:
            line = file.readline()
            lineList = line.split(',')
            print(len(lineList))
            for linePairIndex in range(0, len(lineList), 2):
                self.__predistortionDictionary[float(lineList[linePairIndex+1])] = float(lineList[linePairIndex])
        self.__predistortionList = list(self.__predistortionDictionary.items())
        self.__predistortionList.sort(key=lambda tupleItem : tupleItem[0])


    def __getPosition(self):
        distance = self.__sensor.get_distance()
        y = -(distance - self.__tubeHeight)
        return y
    

    def getY(self):
        return self.__getPosition()

    
    def __deadZone(self, controlValue):
        if controlValue > 0:
            pwmValue = controlValue + 21.7
        elif controlValue < 0:
            pwmValue = controlValue + 21.1
        else:
            pwmValue = 21.4
        return pwmValue


    def __predistort(self, controlValue):
        if controlValue < self.__predistortionList[0][0]:
            pwmValue = self.__predistortionList[0][1]
        elif controlValue >= self.__predistortionList[len(self.__predistortionList)-1][0]:
            pwmValue = self.__predistortionList[len(self.__predistortionList)-1][1]
        else:
            for i in range(len(self.__predistortionList)):
                if controlValue <= self.__predistortionList[i][0]:
                    controlValueDown = self.__predistortionList[i-1][0]
                    controlValueUp = self.__predistortionList[i][0]
                    pwmValueDown = self.__predistortionList[i-1][1]
                    pwmValueUp = self.__predistortionList[i][1]
                    slope = (pwmValueUp - pwmValueDown)/(controlValueUp - controlValueDown)
                    pwmValue = pwmValueDown + slope*(controlValue - controlValueDown)
                    break
        return pwmValue


    def writeControlValue(self, controlValue):
        pwmValue = self.__predistort(controlValue)
        self.__writePWM(pwmValue)

    
    def __writePWM(self, value):
        smoothValue = self.__getSmoothPWM(max(min(value, 100), 0))
        bonnetPwmValue = int(smoothValue*655.35)
        self.__PWMchannel.duty_cycle = (max(min(bonnetPwmValue, 65535), 0))


    def __getSmoothPWM(self, pwmValue):
        listValues = [i * 0.8 for i in range(200)]
    
        index = 0
        for i, val in enumerate(listValues):
            if val > pwmValue:
                index = i
                break

        valueLow = listValues[index-1]
        valueHigh = listValues[index]

        r = random.uniform(valueLow,valueHigh)
        if r < pwmValue:
            return valueHigh
        else:
            return valueLow