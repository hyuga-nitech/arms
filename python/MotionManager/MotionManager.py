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
bendingSensorClose = [2400,2500]
bendingSensorOpen = [1600,2000]

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
        bendingSensorCom2 = [addr for addr in settings if 'bendingSensorCom2' in addr[0]][0][1]
        self.bendingSensorSerialPorts =[addr for addr in settings if 'bendingSensorSerialPorts' in addr [0]][0][1]

        self.bendingSensorip = [bendingSensorCom1,bendingSensorCom2]

        self.bendingSensors  = []

        for i in range(bendingSensorNum):
            bendingSensorManager = BendingSensorManager(ip=self.bendingSensorip[i], port=self.bendingSensorSerialPorts)
            self.bendingSensors.append(bendingSensorManager)

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
        
        self.InitBendingSensorValues    = []

        for i in range(self.bendingSensorNum):
            self.InitBendingSensorValues.append(self.bendingSensors[i].bendingValue)

    def GripperControlValue(self,loopCount: int = 0):
        dictGripperValue = {}
        for i in range(self.bendingSensorNum):
            bendingVal = self.bendingSensors[i].bendingValue
            if bendingSensorClose[i] < bendingSensorOpen[i]:
                bendingValueNorm = (bendingVal - bendingSensorClose[i]) / (self.InitBendingSensorValues[i] - bendingSensorClose[i]) * (targetMax - targetMin) + targetMin
            elif bendingSensorClose[i] > bendingSensorOpen[i]:
                bendingValueNorm = (bendingSensorClose[i] - bendingVal) / (bendingSensorClose[i] - self.InitBendingSensorValues[i]) * (targetMax - targetMin) + targetMin

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
