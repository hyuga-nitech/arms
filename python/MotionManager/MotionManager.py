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
bendingSensorMin = 0
bendingSensorMax = 850

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
        bendingSensorSerialComs = [addr for addr in settings if 'bendingSensorSerialComs' in addr[0]][0][1]
        bendingSensorSerialPorts =[addr for addr in settings if 'bendingSensorSerialPorts' in addr [0]][0][1]

        bendingSensorManager = BendingSensorManager(ip=bendingSensorSerialComs, port=bendingSensorSerialPorts)
        self.bendingSensor = bendingSensorManager

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
        
        self.InitBendingSensorValues    = self.bendingSensor.bendingValue

    def GripperControlValue(self,loopCount: int = 0):
        GripperValue = 0
        bendingVal = self.bendingSensor.bendingValue
        bendingValueNorm = (bendingVal - bendingSensorMin) / (self.InitBendingSensorValues - bendingSensorMin) * (targetMax - targetMin) + targetMin

        if bendingValueNorm > targetMax:
            bendingValueNorm = targetMax
        GripperValue = bendingValueNorm
        
        return GripperValue

    def LocalPosition(self, loopCount: int = 0):
        dictPos = {}
        dictPos = self.optiTrackStreamingManager.position
        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        dictRot = {}
        dictRot = self.optiTrackStreamingManager.rotation
        return dictRot
