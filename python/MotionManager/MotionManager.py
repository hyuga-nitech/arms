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
bendingSensorClose = 2400
bendingSensorOpen = 1600

class MotionManager:
    def __init__(self,defaultRigidBodyNum: int,bendingSensorNum: int) ->None:
        self.defaultRigidBodyNum     = defaultRigidBodyNum
        self.bendingSensorNum        = bendingSensorNum
        self.InitBendingSensorValues = []

        self.optiTrackStreamingManager = OptiTrackStreamingManager(defaultRigidBodyNum=defaultRigidBodyNum)
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        from FileIO.FileIO import FileIO
        fileIO = FileIO()
        settings = fileIO.Read('settings.csv',',')
        bendingSensorCom1 = [addr for addr in settings if 'bendingSensorCom1' in addr[0]][0][1]
        self.bendingSensorSerialPorts =[addr for addr in settings if 'bendingSensorSerialPorts' in addr [0]][0][1]

        self.bendingSensorip = bendingSensorCom1

        bendingSensorManager = BendingSensorManager(ip=self.bendingSensorip, port=self.bendingSensorSerialPorts)
        self.bendingSensors = bendingSensorManager

        # ----- Start receiving bending sensor value ----- #
        bendingSensorThread = threading.Thread(target=bendingSensorManager.StartReceiving)
        bendingSensorThread.setDaemon(True)
        bendingSensorThread.start()
        
        # ----- Set init value ----- #
        self.SetInitialBendingValue()

    def SetInitialBendingValue(self):
        """
        Set init bending value
        """

        self.InitBendingSensorValues = self.bendingSensors.bendingValue

    def GripperControlValue(self,loopCount: int = 0):
        bendingVal = self.bendingSensors.bendingValue
        if bendingSensorClose < bendingSensorOpen:
            bendingValueNorm = (bendingVal - bendingSensorClose) / (self.InitBendingSensorValues - bendingSensorClose) * (targetMax - targetMin) + targetMin
        elif bendingSensorClose[1] > bendingSensorOpen[1]:
            bendingValueNorm = (bendingSensorClose - bendingVal) / (bendingSensorClose - self.InitBendingSensorValues) * (targetMax - targetMin) + targetMin

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