# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Created:         2023/8/1
# Summary:         ratioの値を取得（Bluetooth経由シリアル通信）
# -----------------------------------------------------------------

import time
import serial
import numpy as np

class LEDdirectionManager:
    def __init__(self, port):
        try:
            self.LED_serial = serial.Serial(port, 9600, timeout=0.1)

        except:
            print('Failed to connect to LED')

        self.xbeforeyFlag = 0
        self.xbeforezFlag = 0
        self.mbeforeyFlag = 0
        self.mbeforezFlag = 0

        self.xyFlag = 0
        self.xzFlag = 0
        self.myFlag = 0
        self.mzFlag = 0

        self.listpos1 = []
        self.listpos2 = []

        self.lastsendtime = 0
        self.thresholdVel = 0.5

    def init_send(self):
        try:
            message = '0,0,0,0\n'
            mes = message.encode()
            self.LED_serial.write(mes)

        except:
            print('Failed to send to LED')
    
    def send(self, message):
        try:
            mes = message.encode()
            self.LED_serial.write(mes)
            self.lastsendtime = time.perf_counter()

        except:
            print('Failed to send to LED')

    def flagchecker(self,position):
        self.sendFlag = 0
        message = ''

        if ((time.perf_counter() - self.lastsendtime) > 0.05):
            pos1 = position['RigidBody1'] * 1000
            pos2 = position['RigidBody2'] * 1000

            self.listpos1.append(pos1)
            self.listpos2.append(pos2)

            if len(self.listpos1) == 2:
                if (self.xbeforeyFlag != 1) and (self.listpos1[1][0] - self.listpos1[0][0] < (-1 * self.thresholdVel)):
                    self.xyFlag = 1
                    self.xbeforeyFlag = 1
                    self.sendFlag = 1
                elif(self.xbeforeyFlag != -1) and (self.listpos1[1][0] - self.listpos1[0][0] > self.thresholdVel):
                    self.xyFlag = -1
                    self.xbeforeyFlag = -1
                    self.sendFlag = 1
                elif(self.xbeforeyFlag != 0) and ((-1 * self.thresholdVel) <= (self.listpos1[1][0] - self.listpos1[0][0]) <= self.thresholdVel):
                    self.xyFlag = 0
                    self.xbeforeyFlag = 0
                    self.sendFlag = 1

                if (self.xbeforezFlag != 1) and (self.listpos1[1][1] - self.listpos1[0][1] > self.thresholdVel):
                    self.xzFlag = 1
                    self.xbeforezFlag = 1
                    self.sendFlag = 1
                elif(self.xbeforezFlag != -1) and (self.listpos1[1][1] - self.listpos1[0][1] < (-1 * self.thresholdVel)):
                    self.xzFlag = -1
                    self.xbeforezFlag = -1
                    self.sendFlag = 1
                elif(self.xbeforezFlag != 0) and ((-1 * self.thresholdVel) <= (self.listpos1[1][1] - self.listpos1[0][1]) <= self.thresholdVel):
                    self.xzFlag = 0
                    self.xbeforezFlag = 0
                    self.sendFlag = 1

                if (self.mbeforeyFlag != 1) and (self.listpos2[1][0] - self.listpos2[0][0] < (-1 * self.thresholdVel)):
                    self.myFlag = 1
                    self.mbeforeyFlag = 1
                    self.sendFlag = 1
                elif(self.mbeforeyFlag != -1) and (self.listpos2[1][0] - self.listpos2[0][0] > self.thresholdVel):
                    self.myFlag = -1
                    self.mbeforeyFlag = -1
                    self.sendFlag = 1
                elif(self.mbeforeyFlag != 0) and ((-1 * self.thresholdVel) <= (self.listpos2[1][0] - self.listpos2[0][0]) <= self.thresholdVel):
                    self.myFlag = 0
                    self.mbeforeyFlag = 0
                    self.sendFlag = 1

                if (self.mbeforezFlag != 1) and (self.listpos2[1][1] - self.listpos2[0][1] > self.thresholdVel):
                    self.mzFlag = 1
                    self.mbeforeyFlag = 1
                    self.sendFlag = 1
                elif(self.mbeforezFlag != -1) and (self.listpos2[1][1] - self.listpos2[0][1] < (-1 * self.thresholdVel)):
                    self.mzFlag = -1
                    self.mbeforeyFlag = -1
                    self.sendFlag = 1
                elif(self.mbeforezFlag != 0) and ((-1 * self.thresholdVel) <= (self.listpos2[1][1] - self.listpos2[0][1]) <= self.thresholdVel):
                    self.mzFlag = 0
                    self.mbeforeyFlag = 0
                    self.sendFlag = 1

                message = str(int(self.xyFlag)) + "," + str(int(self.xzFlag)) + "," + str(int(self.myFlag)) + "," + str(int(self.mzFlag)) + "\n"

                del self.listpos1[0]
                del self.listpos2[0]

        return self.sendFlag, message
