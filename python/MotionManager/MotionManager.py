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
targetMin        = 200
targetMax        = 240
bendingSensorClose = 0
bendingSensorOpen = 1

class MotionManager:
    def __init__(self,defaultRigidBodyNum: int) ->None:
        self.defaultRigidBodyNum     = defaultRigidBodyNum
        self.InitBendingSensorValues = []

        self.optiTrackStreamingManager = OptiTrackStreamingManager(defaultRigidBodyNum=defaultRigidBodyNum)
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        from FileIO.FileIO import FileIO
        fileIO = FileIO()
        settings = fileIO.Read('settings.csv',',')
        self.bendingSensorPort = [addr for addr in settings if 'bendingSensorPort1' in addr[0]][0][1]
        self.bendingSensorSerialBaudrate = [addr for addr in settings if 'bendingSensorSerialBaudrate' in addr [0]][0][1]

        self.bendingSensorManager = BendingSensorManager(port=self.bendingSensorPort, baudrate=self.bendingSensorSerialBaudrate)

        # ----- Start receiving bending sensor value ----- #
        bendingSensorThread = threading.Thread(target=self.bendingSensorManager.StartReceiving)
        bendingSensorThread.setDaemon(True)
        bendingSensorThread.start()

    def GripperControlValue(self,loopCount: int = 0):
        bendingVal = self.bendingSensorManager.bendingValue
        if bendingSensorClose < bendingSensorOpen:
            bendingValueNorm = (bendingVal - bendingSensorClose) / (bendingSensorOpen - bendingSensorClose) * (targetMax - targetMin) + targetMin
        elif bendingSensorClose > bendingSensorOpen:
            bendingValueNorm = (bendingSensorClose - bendingVal) / (bendingSensorClose - bendingSensorOpen) * (targetMax - targetMin) + targetMin

        if bendingValueNorm > targetMax:
            bendingValueNorm = targetMax
    
        return bendingValueNorm

    def LocalPosition(self, loopCount: int = 0):
        dictPos = {}
        dictPos = self.optiTrackStreamingManager.position
        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        dictRot = {}
        dictRot = self.optiTrackStreamingManager.rotation
        return dictRot
