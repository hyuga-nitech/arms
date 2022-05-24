# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         RigidBody座標の取得
# -----------------------------------------------------------------

import threading
import time
import numpy as np
import csv

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from BendingSensor.BendingSensorManager import BendingSensorManager

# ----- Numeric range remapping ----- #
targetMin=100
targetMax=850
originalMin = 0
originalMax = 1
bendingSensorMin = 0
bendingSensorMax = 850

class MotionManager:
    def __int__(self,defaultRigidBodyNum: int,bendingSensorNum: int = 1) ->None:
        self.defaultRigidBodyNum     = defaultRigidBodyNum
        self.bendingSensorNum        = bendingSensorNum
        self.InitBendingSensorValues = []

        self.optiTrackStreamingManager = OptiTrackStreamingManager(defaultRigidBodyNum=defaultRigidBodyNum)
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        self.bendingSensors         = []
        bendingSensorSerialComs     = ['COM11']
        bendingSensorSerialPorts    = [9600]

        for i in range(bendingSensorNum):
            bendingSensorManager = BendingSensorManager(ip=bendingSensorSerialComs[i], port=bendingSensorSerialPorts[i])
            self.bendingSensors.append(bendingSensorManager)

            # ----- Start receiving bending sensor value from UDP socket ----- #
            bendingSensorThread = threading.Thread(target=bendingSensorManager.StartReceiving)
            bendingSensorThread.setDaemon(True)
            bendingSensorThread.start()
        
        # ----- Set init value ----- #
            self.SetInitialBendingValue()

    def SetInitialBendingValue(self):
        """
        Set init bending value
        """
        
        if self.gripperInputSystem == 'bendingsensor':
            self.InitBendingSensorValues    = []

            for i in range(self.bendingSensorNum):
                self.InitBendingSensorValues.append(self.bendingSensors[i].bendingValue)

    def GripperControlValue(self,loopCount: int = 0):
        dictGripperValue = {}
        for i in range(self.bendingSensorNum):
            bendingVal = self.bendingSensors[i].bendingValue
            bendingValueNorm = (bendingVal - bendingSensorMin) / (self.InitBendingSensorValues[i] - bendingSensorMin) * (targetMax - targetMin) + targetMin

            if bendingValueNorm > targetMax:
                bendingValueNorm = targetMax
            dictGripperValue['gripperValue'+str(i+1)] = bendingValueNorm
        
        return dictGripperValue