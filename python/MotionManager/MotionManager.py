# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         RigidBody座標の取得
# -----------------------------------------------------------------

import threading
import numpy as np

# ----- Custom class ----- #
from Optitrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from BendingSensor.BendingSensorManager import BendingSensorManager

# ----- Numeric range remapping ----- #
targetMin        = 180
targetMax        = 240
bendingSensorClose = 0
bendingSensorOpen = 1

class MotionManager:
    def __init__(self,rigidBodyNum: int,bendingSensorNum: int,bendingSensorSerialPorts,bendingSensorSerialBaudrates) ->None:
        self.bendingSensorNum        = bendingSensorNum
        self.InitBendingSensorValues = []

        self.optiTrackStreamingManager = OptiTrackStreamingManager(rigidBodyNum=rigidBodyNum)
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        self.bendingSensors                = []
        self.bendingSensorSerialPorts      = bendingSensorSerialPorts
        self.bendingSensorSerialBaudrates  = bendingSensorSerialBaudrates

        for i in range(self.bendingSensorNum):
            bendingSensorManager = BendingSensorManager(port=self.bendingSensorSerialPorts[i], baudrate=self.bendingSensorSerialBaudrates[i])
            self.bendingSensors.append(bendingSensorManager)

            # ----- Start receiving bending sensor value from UDP socket ----- #
            bendingSensorThread = threading.Thread(target=bendingSensorManager.StartReceiving)
            bendingSensorThread.setDaemon(True)
            bendingSensorThread.start()

    def GripperControlValue(self,loopCount: int = 0):
        dictGripperValue = {}
        for i in range(self.bendingSensorNum):
            bendingVal = self.bendingSensors[i].bendingValue
            if bendingSensorClose < bendingSensorOpen:
                bendingValueNorm = (bendingVal - bendingSensorClose) / (bendingSensorOpen - bendingSensorClose) * (targetMax - targetMin) + targetMin
            elif bendingSensorClose > bendingSensorOpen:
                bendingValueNorm = (bendingSensorClose - bendingVal) / (bendingSensorClose - bendingSensorOpen) * (targetMax - targetMin) + targetMin

            if bendingValueNorm > targetMax:
                bendingValueNorm = targetMax
            dictGripperValue['gripperValue'+str(i+1)] = bendingValueNorm

        return dictGripperValue

    def LocalPosition(self, loopCount: int = 0):
        dictPos = {}
        dictPos = self.optiTrackStreamingManager.position
        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        dictRot = {}
        dictRot = self.optiTrackStreamingManager.rotation
        return dictRot
