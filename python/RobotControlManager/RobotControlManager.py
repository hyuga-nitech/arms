# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/24
# Summary:         xArm,mikataArm,BendingSensorの制御
# -----------------------------------------------------------------

import time
import threading
import numpy as np
from ctypes import windll

# ----- Custom class ----- #
from mikataArm.mikataArmTransform import mikataTransform
from mikataArm.mikataControl import mikataControl
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from FileIO.FileIO import FileIO

# ----- Setting: Number ----- #
defaultRigidBodyNum     = 1
mikataMovingLimit       = 1000

class RobotControlManager:
    def __init__(self) ->None:
        pass

    def SendDataToRobot(self,executionTime: int = 120, isExportData: bool = True, isEnableArm: bool = True):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour           = MotionBehaviour(defaultRigidBodyNum)
        mikatatransform     = mikataTransform()
        motionManager       = MotionManager(defaultRigidBodyNum)
        mikatacontrol       = mikataControl()
        dataRecordManager   = DataRecordManager(RigidBodyNum=defaultRigidBodyNum)

        if isEnableArm:
            self.InitializeAll(mikatatransform, mikatacontrol)

        # ----- Control flags ----- #
        isMoving = False
        isRatio  = True

        try:
            while True:
                # if time.perf_counter() - taskStartTime > executionTime:
                #     # ----- Exit processing after task time elapses ----- #
                #     isMoving    = False

                #     self.taskTime.append(time.perf_counter() - taskStartTime)
                #     self.PrintProcessInfo()

                #     # ----- Export recorded data ----- #
                #     if isExportData:
                #         dataRecordManager.ExportSelf()

                #     if isEnableArm:
                #         arm.disconnect()
                #         mikatacontrol.ClosePort()

                #     print('----- Finish task -----')
                #     break

                if isMoving:
                    # ----- Get transform data ----- #
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)

                    mikataPosition   = Behaviour.GetmikataArmTransform(localPosition)

                    mikataPosition = mikataPosition * 1000

                    # ----- Set mikata transform ----- #
                    mikatatransform.x, mikatatransform.y, mikatatransform.z     = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                    # ----- Bending sensor ----- #
                    BendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)

                    # ----- Calculate mikata Current ----- #
                    mikataC1, mikataC2, mikataC3, mikataC4 = mikatatransform.Transform()
                    mikataC5                               = mikatatransform.Degree2Current(BendingValue)

                    # ----- Safety Check -----#
                    mikatadiffC1 = mikataC1 - beforeC1
                    mikatadiffC2 = mikataC2 - beforeC2
                    mikatadiffC3 = mikataC3 - beforeC3
                    mikatadiffC4 = mikataC4 - beforeC4
                    mikatadiffC5 = mikataC5 - beforeC5
                    beforeC1, beforeC2, beforeC3, beforeC4, beforeC5 = mikataC1, mikataC2, mikataC3, mikataC4, mikataC5

                    if abs(mikatadiffC1) > mikataMovingLimit or abs(mikatadiffC2) > mikataMovingLimit or abs(mikatadiffC3) > mikataMovingLimit or abs(mikatadiffC4) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm. Please enter "r" to reset xArm, or "q" to quit')
                    elif abs(mikatadiffC5) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm Gripper. Please enter "r" to reset xArm, or "q" to quit')
                    else:
                        if isEnableArm:
                            # ----- Send to Arms ----- #
                            mikatacontrol.dxl_goal_position = [mikataC1, mikataC2, mikataC3, mikataC4, mikataC5]

                    # ----- Data recording ----- #
                    dataRecordManager.Record(localPosition)

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "s": start control \n')

                    # ----- FIX ME !!! ----- #
                    # "r": Clean error and init arm, can't use because of "Port is in Use" error.


                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnableArm:
                            mikatacontrol.ClosePort()

                        self.PrintProcessInfo()
                        break

                    # # ----- Reset xArm and gripper ----- #
                    # elif keycode == 'r':
                    #     if isEnableArm:
                    #         self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())
                        
                        mikataPosition = Behaviour.GetmikataArmTransform(motionManager.LocalPosition())
                        
                        mikataPosition = mikataPosition * 1000

                        # ----- Set mikata transform ----- #
                        mikatatransform.x, mikatatransform.y, mikatatransform.z     = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                        # ----- Bending sensor ----- #
                        BendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                        
                        beforeC1, beforeC2, beforeC3, beforeC4 = mikatatransform.Transform()
                        beforeC5                               = mikatatransform.Degree2Current(BendingValue)

                        motionManager.SetInitialBendingValue()

                        isMoving    = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()
            
            if isExportData:
                dataRecordManager.ExportSelf()

            if isEnableArm:
                mikatacontrol.ClosePort()

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

    def InitializeAll(self, mikatatransform, mikatacontrol, isSetInitPosition = True):
        # ----- mikata Arm ----- #
        mikatacontrol.OpenPort()
        if isSetInitPosition:
            mikatacontrol.dxl_goal_position = mikatatransform.GetmikataInitialTransform()
            mikataThread = threading.Thread(target=mikatacontrol.SendtomikataArm)
            mikataThread.setDaemon(True)
            mikataThread.start()
        print('Initialized > mikataArm')

    def PrintProcessInfo(self):
        """
        Print process information. 
        """

        print('----- Process info -----')
        print('Total loop count > ', self.loopCount)
        for ttask in self.taskTime:
            print('Task time\t > ', ttask, '[s]')
        print('------------------------')